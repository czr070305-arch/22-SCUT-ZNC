#include "pid.h"

//pid初始化
void PID_Init(PID_t *p, float Kp, float Ki, float Kd,
              float out_max, float int_max, float dead_zone, uint8_t mode)
{
    // 设置可调参数
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->out_max = out_max;
    p->int_max = int_max;
    p->dead_zone = dead_zone;
    p->mode = mode;

    // 清零所有历史状态（防止首次计算时使用未定义值）
    p->target_val = 0.0f;
    p->actual_val = 0.0f;
    p->err = 0.0f;
    p->err_last = 0.0f;
    p->err_llast = 0.0f;
    p->err_sum = 0.0f;
    p->out_last = 0.0f;
    p->out = 0.0f;
}

//pid计算
float PID_Calc(PID_t *p, float target, float feedback)
{
    // 记录目标值和反馈值（供 VOFA+ 调试）
    p->target_val = target;
    p->actual_val = feedback;

    // 计算当前误差 e(k) = 目标值 - 反馈值
    p->err = target - feedback;

    // 死区处理：误差小于死区时视为 0，防止执行器微抖
    if (fabsf(p->err) < p->dead_zone)
    {
        p->err = 0.0f;
    }

    /////////////////////////  位置式 PID  /////////////////////////
    //
    //           ┌─ 积分项（带 anti-windup）
    //           │
    //   u(k) = Kp·e(k) + Ki·Σe(k) + Kd·[e(k) - e(k-1)]
    //                               │
    //                               └─ 微分项（误差的变化量）

    if (p->mode == PID_MODE_POS)
    {

        p->err_sum += p->err;

        // 积分限幅：防止积分饱和（motor 启动/停止时关键）
        // int_max 为 0 表示不限幅, 不加判断的话会把 err_sum 强制归零, 积分失效
        if (p->int_max > 0.0f)
        {
            if (p->err_sum >  p->int_max) p->err_sum =  p->int_max;
            if (p->err_sum < -p->int_max) p->err_sum = -p->int_max;
        }

        p->out = p->Kp * p->err
               + p->Ki * p->err_sum
               + p->Kd * (p->err - p->err_last);
    }

    /////////////////////////  增量式 PID  /////////////////////////
    //
    //   Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
    //
    //   特点：输出是增量，启动时从 0 开始累加，不会跳变。
    //         适合速度环等一阶对象控制。

    else
     {

        float delta = p->Kp * (p->err - p->err_last)
                    + p->Ki * p->err
                    + p->Kd * (p->err - 2.0f * p->err_last + p->err_llast);

        p->out += delta;
    }

    // 输出限幅：保护执行器不超过安全范围
    // out_max 为 0 表示不限幅, 不加判断的话输出会被钳死在 0
    if (p->out_max > 0.0f)
    {
        if (p->out >  p->out_max) p->out =  p->out_max;
        if (p->out < -p->out_max) p->out = -p->out_max;
    }

    // 更新历史状态为下一周期准备
    p->err_llast = p->err_last;
    p->err_last  = p->err;
    p->out_last  = p->out;

    return p->out;
}

//重置 PID 历史状态
void PID_Reset(PID_t *p)
{
    // 清零所有历史状态，保留可调参数不变
    p->target_val = 0.0f;
    p->actual_val = 0.0f;
    p->err        = 0.0f;
    p->err_last   = 0.0f;
    p->err_llast  = 0.0f;
    p->err_sum    = 0.0f;
    p->out_last   = 0.0f;
    p->out        = 0.0f;
}

//在线调 PID 参数
void PID_SetTuning(PID_t *p, float Kp, float Ki, float Kd)
{
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
}
