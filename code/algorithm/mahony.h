#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdint.h>

/*
 * mahony.h — Mahony 自适应四元数姿态解算
 *
 * !!!换 MPU6050、ICM20948 需要改量程才能复用(一般在工程的imu.h)：IMU_ACC_SENS_G 和 IMU_GYRO_SENS_DPS
 * 
 * 陀螺仪积分短期准、长期漂；加速度计测重力方向长期稳、短期被振动干扰。
 * Mahony 用加速度计算出的重力方向去修正陀螺仪积分出的姿态，修正量经 PI
 * 反馈到角速度上，再用修正后的角速度更新四元数。
 *
 * 用四元数而非直接积欧拉角：欧拉角在 pitch = ±90° 附近有万向锁，
 * 小车过减速带颠簸时 pitch 可能很大，用欧拉角会算飞。
 *
 * 本模块不包含任何芯片相关代码，只接收 int16 原始值，
 * 
 */

typedef struct {
    float roll;         // 横滚角，度，±180
    float pitch;        // 俯仰角，度，±90
    float yaw;          // 航向角，度，[0,360)，无磁力计参考会缓慢漂移
    float q[4];         // 四元数，调试用
} imu_t;

void mahony_init(void);                                       // 复位四元数和全部历史状态
void mahony_update(const int16_t acc[3], const int16_t gyro[3],
                     float dt, imu_t *out);                // dt 为实际采样间隔，单位秒
void mahony_reset_yaw(void);                                  // 当前航向设为零点，不动四元数

#endif
