#include "mahony.h"
#include "hw_imu.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/*
 * mahony.c — Mahony 四元数姿态解算实现
 *
 * 移植自 TI_Learning/MSPM0G3507/06_Project_26/Algorithm/Mahony.c，
 * 改动：灵敏度直接用 hw_imu.h 的常量，移除 SetZero。
 *
 *   - Kp 修正姿态收敛速度，越大越快但越容易被振动带偏
 *   - Ki 跟踪陀螺仪零偏，慢速积分，只在近似静止时累积
 *   - 快速转动时冻结积分，否则加速度计测到的离心力会被当成重力方向
 */

/* ══════════════════════════════════════════════════════════════
 *  内部状态
 * ══════════════════════════════════════════════════════════════ */

/* 全部静态变量，模块只能有一个实例。本项目只有一颗 IMU，够用。 */
static struct {
    float    q0, q1, q2, q3;        // 当前姿态四元数
    float    integral_fb[3];        // PI 积分项，跟踪陀螺仪零偏（rad/s）
    uint16_t init_count;            // 前 400 次用大 Kp 快速收敛
    float    yaw_unwrapped;         // 解包裹后的连续 yaw
    float    yaw_offset;            // yaw 归零偏移
    float    last_raw_yaw;          // 上次输出的原始 yaw（±180），解包裹用
    uint8_t  first_run;
} mstate;

/* ══════════════════════════════════════════════════════════════
 *  内部工具
 * ══════════════════════════════════════════════════════════════ */

/*
 * inv_sqrt — 快速逆平方根（Quake III 算法）
 *
 * 1/sqrtf() 在 Cortex-M4 上要几百周期，这个只要十几次乘法，精度 0.17%。
 * 用 memcpy 而非指针强转，是为了避开严格别名规则，否则 -O2 下会被优化掉。
 * 不加 inline：Keil AC5 默认 C90，inline 不是关键字。
 */
static float inv_sqrt(float x)
{
    float    halfx = 0.5f * x;
    float    y     = x;
    uint32_t i;

    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/* ══════════════════════════════════════════════════════════════
 *  对外接口
 * ══════════════════════════════════════════════════════════════ */

/*
 * mahony_init — 复位解算器
 *
 * 四元数恢复到 [1,0,0,0]（无旋转），清空积分项、yaw 偏移和零偏追踪历史。
 * 上电或重新校准后调用，让姿态从零开始重新收敛。
 */
void mahony_init(void)
{
    mstate.q0 = 1.0f;               // [1,0,0,0] 表示无旋转，初始姿态即零点
    mstate.q1 = 0.0f;
    mstate.q2 = 0.0f;
    mstate.q3 = 0.0f;

    mstate.integral_fb[0] = 0.0f;
    mstate.integral_fb[1] = 0.0f;
    mstate.integral_fb[2] = 0.0f;

    mstate.init_count    = 0U;
    mstate.yaw_unwrapped = 0.0f;
    mstate.yaw_offset    = 0.0f;
    mstate.last_raw_yaw  = 0.0f;

    /* first_run 让第一次 update 直接写入 yaw_unwrapped，
     * 避免从 0 开始解包裹时因为初始角度不为 0 而算出巨大跳变 */
    mstate.first_run = 1U;
}

/*
 * mahony_update — Mahony 单步姿态解算
 *
 * 输入已减零偏的原始值，跑一次完整的 PI 反馈 + 四元数更新。
 * 内部按 9 个步骤执行，顺序不能乱：物理量转换 → 自适应增益 →
 * 重力误差叉乘 → PI 修正角速度 → 四元数微分 → 归一化 →
 * 转欧拉角 → yaw 解包裹 → 输出。
 *
 * dt 必须是实际采样间隔，用固定常数会随任务周期抖动一起偏，
 * yaw 累积误差按同比例放大。
 */
void mahony_update(const int16_t acc[3], const int16_t gyro[3],
                     float dt, imu_t *out)
{
    float ax, ay, az;
    float gx, gy, gz;
    float acc_mag;
    float kp, ki;
    float recip;
    float vx, vy, vz;
    float ex, ey, ez;
    float qd0, qd1, qd2, qd3;
    float norm;
    float cur_roll, cur_pitch, cur_yaw;
    float diff;

    if (out == NULL) {
        return;
    }

    /* 1. 原始值 → 物理量。加速度单位 g（只用方向不用绝对值），
     *    角速度单位 rad/s（四元数微分方程的要求）。 */
    ax = (float)acc[0] / IMU_ACC_SENS_G;
    ay = (float)acc[1] / IMU_ACC_SENS_G;
    az = (float)acc[2] / IMU_ACC_SENS_G;

    gx = ((float)gyro[0] / IMU_GYRO_SENS_DPS) * 0.017453292f;
    gy = ((float)gyro[1] / IMU_GYRO_SENS_DPS) * 0.017453292f;
    gz = ((float)gyro[2] / IMU_GYRO_SENS_DPS) * 0.017453292f;

    /* 2. 加速度幅值。静止时应为 1.0（一个重力加速度），明显偏离说明
     *    车在加速、颠簸或碰撞，此时加速度计测的不只是重力。 */
    acc_mag = ax * ax + ay * ay + az * az;

    /* 3. 自适应增益。初始化阶段 Kp=8 快速收敛；稳定后幅值在
     *    [0.8g,1.2g] 内认为近似静止（Kp=4.8 多信加速度计），
     *    超出认为剧烈运动（Kp=3.6 多信陀螺仪）。
     *    判据用平方值省掉开方：0.8²=0.64，1.2²=1.44。 */
    if (mstate.init_count < 400U) {
        mstate.init_count++;
        kp = 8.0f;
        ki = 0.002f;
    } else {
        if ((acc_mag > 1.44f) || (acc_mag < 0.64f)) {
            kp = 3.6f;
            ki = 0.001f;
        } else {
            kp = 4.8f;
            ki = 0.0015f;
        }
    }

    /* 4. 重力方向误差 */
    if (acc_mag > 0.01f) {
        recip = inv_sqrt(acc_mag);

        ax *= recip;                // 归一化，得到实测重力方向单位向量
        ay *= recip;
        az *= recip;

        /* 由当前四元数推算「如果姿态正确，重力方向应该是多少」，
         * 这三行是四元数旋转矩阵的第三行（重力在机体系下的投影） */
        vx = 2.0f * (mstate.q1 * mstate.q3 - mstate.q0 * mstate.q2);
        vy = 2.0f * (mstate.q0 * mstate.q1 + mstate.q2 * mstate.q3);
        vz = mstate.q0 * mstate.q0 - mstate.q1 * mstate.q1
           - mstate.q2 * mstate.q2 + mstate.q3 * mstate.q3;

        /* 叉乘 = 实测方向 × 预测方向，方向是该往哪个轴转，模长是转多少 */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        /* 5. 积分项跟踪陀螺仪零偏。快速转动时加速度计测到的离心力
         *    会被误认为重力方向，继续积分会越积越大（表现为转一圈回来
         *    姿态歪掉），所以角速度超过 0.5 rad/s（约 28°/s）时冻结。
         *    积分项冻结期间仍然生效，只是不再累积。 */
        if ((fabsf(gx) < 0.5f) && (fabsf(gy) < 0.5f) && (fabsf(gz) < 0.5f)) {
            mstate.integral_fb[0] += ex * dt;
            mstate.integral_fb[1] += ey * dt;
            mstate.integral_fb[2] += ez * dt;
        }

        gx += ki * mstate.integral_fb[0];
        gy += ki * mstate.integral_fb[1];
        gz += ki * mstate.integral_fb[2];

        gx += kp * ex;              // 比例项，快速修正
        gy += kp * ey;
        gz += kp * ez;
    }

    /* 6. 四元数微分方程 q̇ = 0.5·q⊗ω，ω 是修正后的角速度 */
    qd0 = 0.5f * (-mstate.q1 * gx - mstate.q2 * gy - mstate.q3 * gz);
    qd1 = 0.5f * ( mstate.q0 * gx + mstate.q2 * gz - mstate.q3 * gy);
    qd2 = 0.5f * ( mstate.q0 * gy - mstate.q1 * gz + mstate.q3 * gx);
    qd3 = 0.5f * ( mstate.q0 * gz + mstate.q1 * gy - mstate.q2 * gx);

    mstate.q0 += qd0 * dt;
    mstate.q1 += qd1 * dt;
    mstate.q2 += qd2 * dt;
    mstate.q3 += qd3 * dt;

    /* 7. 归一化。数值积分会让模长慢慢偏离 1，每步校正一次 */
    norm = inv_sqrt(mstate.q0 * mstate.q0 + mstate.q1 * mstate.q1
                  + mstate.q2 * mstate.q2 + mstate.q3 * mstate.q3);

    mstate.q0 *= norm;
    mstate.q1 *= norm;
    mstate.q2 *= norm;
    mstate.q3 *= norm;

    /* 8. 四元数 → 欧拉角，旋转顺序 Z-Y-X。
     *    pitch 用 asinf 得到 ±90° 范围，正好是俯仰角的物理极限。 */
    cur_roll  = atan2f(2.0f * (mstate.q0 * mstate.q1 + mstate.q2 * mstate.q3),
                       1.0f - 2.0f * (mstate.q1 * mstate.q1 + mstate.q2 * mstate.q2)) * 57.29578f;
    cur_pitch = asinf(2.0f * (mstate.q0 * mstate.q2 - mstate.q3 * mstate.q1)) * 57.29578f;
    cur_yaw   = atan2f(2.0f * (mstate.q0 * mstate.q3 + mstate.q1 * mstate.q2),
                       1.0f - 2.0f * (mstate.q2 * mstate.q2 + mstate.q3 * mstate.q3)) * 57.29578f;

    /* 9. yaw 解包裹。atan2f 在 ±180° 处会突变（179° 转一点变成 -179°），
     *    直接拿去用航向环会以为车猛地转了 358°。做法是只累加相邻两次的
     *    差值，差值先归一化到 ±180° 以内，这样连续转多圈也不会跳变。 */
    if (mstate.first_run != 0U) {
        mstate.yaw_unwrapped = cur_yaw;
        mstate.last_raw_yaw  = cur_yaw;
        mstate.first_run     = 0U;
    } else {
        diff = cur_yaw - mstate.last_raw_yaw;

        while (diff > 180.0f) {
            diff -= 360.0f;
        }
        while (diff < -180.0f) {
            diff += 360.0f;
        }

        mstate.yaw_unwrapped += diff;
        mstate.last_raw_yaw   = cur_yaw;
    }

    out->roll  = cur_roll;
    out->pitch = cur_pitch;

    out->yaw = mstate.yaw_unwrapped - mstate.yaw_offset;
    while (out->yaw < 0.0f) {
        out->yaw += 360.0f;
    }
    while (out->yaw >= 360.0f) {
        out->yaw -= 360.0f;
    }

    out->q[0] = mstate.q0;
    out->q[1] = mstate.q1;
    out->q[2] = mstate.q2;
    out->q[3] = mstate.q3;
}

/* 把当前解包裹后的 yaw 记成偏移量，四元数不动所以 roll/pitch 不受影响 */
void mahony_reset_yaw(void)
{
    mstate.yaw_offset = mstate.yaw_unwrapped;
}
