#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>
#include <math.h>

/*
 * PID.h — 通用 PID 底层算法
 *
 * 支持位置式（PID_MODE_POS）和增量式（PID_MODE_INC）两种模式，
 * 通过 PID_t.mode 字段切换，共用 PID_Calc() 函数。
 *
 * 位置式（MODE_POS）：输出 = Kp·e + Ki·Σe + Kd·Δe
 *   适用场景：角度环、转向环、直行纠偏环 —— 输出是绝对控制量
 *
 * 增量式（MODE_INC）：输出 += Kp·Δe + Ki·e + Kd·Δ?0?5e
 *   适用场景：速度环 —— 输出是控制量的增量，启动平滑不跳变
 */

// PID 模式
#define PID_MODE_POS    0   // 位置式 PID
#define PID_MODE_INC    1   // 增量式 PID

typedef struct {
    // 可调参数（由用户根据控制对象整定）
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float out_max;          // 输出限幅（绝对值上限，保护执行器）
    float int_max;          // 积分限幅（anti-windup，位置式有效）
    float dead_zone;        // 死区（误差小于此值时认为误差=0，防止微抖）

    uint8_t mode;           // PID_MODE_POS 或 PID_MODE_INC

    // 状态记录（供 VOFA+ 在线调试查看）
    float target_val;       // 本周期目标值
    float actual_val;       // 本周期反馈值

    // 内部历史状态（PID_Reset 清零）
    float err;              // e(k)   — 当前偏差
    float err_last;         // e(k-1) — 上次偏差
    float err_llast;        // e(k-2) — 上上次偏差（增量式 Δ?0?5e 需要）
    float err_sum;          // Σe     — 误差累计（位置式积分需要）
    float out_last;         // u(k-1) — 保留字段
    float out;              // 当前输出
} PID_t;

/////////////////////////  公开 API  /////////////////////////

/* ════════════════════════════════════════════════════════════════
 * PID_Init — 初始化 PID 实例
 *
 * 【输入】 p:         PID 实例指针
 *          Kp,Ki,Kd:  PID 三参数
 *          out_max:   输出限幅（>0），设为 0 不限幅
 *          int_max:   积分限幅（>0），设为 0 不限幅（仅位置式有效）
 *          dead_zone: 死区，设为 0 无死区
 *          mode:      PID_MODE_POS 或 PID_MODE_INC
 *
 * 详细说明：
 *   - 清零所有历史状态，防止首次计算时使用未定义值
 * ════════════════════════════════════════════════════════════════ */
void  PID_Init(PID_t *p, float Kp, float Ki, float Kd,
               float out_max, float int_max, float dead_zone, uint8_t mode);

/* ════════════════════════════════════════════════════════════════
 * PID_Calc — PID 核心计算（位置式/增量式自动选择）
 *
 * 【输入】 p:        PID 实例指针
 *          target:   目标值
 *          feedback: 反馈值
 * 【输出】 float: 控制输出量
 *
 * 详细说明：
 *   - 位置式（PID_MODE_POS）：out = Kp·e + Ki·Σe + Kd·(e - e_last)
 *   - 增量式（PID_MODE_INC）：Δu = Kp·Δe + Ki·e + Kd·Δ?0?5e
 *   - 死区：|err| < dead_zone 时视为 0，防止执行器微抖
 *   - 输出限幅：|out| 不超过 out_max
 * ════════════════════════════════════════════════════════════════ */
float PID_Calc(PID_t *p, float target, float feedback);

/* ════════════════════════════════════════════════════════════════
 * PID_Reset — 重置 PID 历史状态（保留参数不变）
 *
 * 【输入】 p: PID 实例指针
 *
 * 详细说明：
 *   - 在控制模式切换（如直行→转弯）时调用
 * ════════════════════════════════════════════════════════════════ */
void  PID_Reset(PID_t *p);

/* ════════════════════════════════════════════════════════════════
 * PID_SetTuning — 在线调整 PID 参数
 *
 * 【输入】 p:         PID 实例指针
 *          Kp,Ki,Kd:  新的 PID 参数
 *
 * 详细说明：
 *   - 用于 VOFA+ 下行命令调参，无需重启即可改变控制行为
 * ════════════════════════════════════════════════════════════════ */
void  PID_SetTuning(PID_t *p, float Kp, float Ki, float Kd);

#endif
