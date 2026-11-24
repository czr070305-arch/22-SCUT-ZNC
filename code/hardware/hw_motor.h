#ifndef _HW_MOTOR_H_
#define _HW_MOTOR_H_

#include "zf_common_headfile.h"

/*
 * 8701 双路电机驱动
 *
 * 驱动芯片 DRV8701E 是 PH/EN 接口:
 *   PH  只给高低电平, 决定 H 桥导通方向 -> 正转/反转
 *   EN  给 PWM, 占空比决定转速; EN=0 时两个下管导通 = 刹车
 *
 * 接线(必须逐根连, 不能拿四芯排线直插):
 *   主板 H3.1 (L1/PE9)  -> 驱动板 H1.5 (EN1-IN)  左调速
 *   主板 H3.2 (L2/PE11) -> 驱动板 H1.1 (PH1-IN)  左方向
 *   主板 H3.3 (R1/PE13) -> 驱动板 H1.2 (PH2-IN)  右方向
 *   主板 H3.4 (R2/PE14) -> 驱动板 H1.6 (EN2-IN)  右调速
 *   主板上任意 GND      -> 驱动板 H1.3 或 H1.4   共地, 漏了信号就没有参考电平
 *
 * 定时器占用: TIM1 的 CH1 和 CH4
 *   E11/E13 虽然也被 TIM1 重映射覆盖, 但这里配成普通推挽输出,
 *   输出的是 GPIO 电平, 与 PWM 无关, 不需要单独占用定时器通道
 */

//左电机
#define MOTOR_L_PH_PIN      (E11)                        //方向, 普通GPIO
#define MOTOR_L_EN_PIN      (TIM1_PWM_MAP3_CH1_E9)     //调速, PWM

//右电机
#define MOTOR_R_PH_PIN      (E13)                       //方向, 普通GPIO
#define MOTOR_R_EN_PIN      (TIM1_PWM_MAP3_CH4_E14)     //调速, PWM

//20kHz 超出人耳范围, 跑起来没有啸叫; DRV8701 支持到 100kHz
#define MOTOR_PWM_FREQ      (20000)

//占空比上限, 与逐飞库 PWM_DUTY_MAX 一致(10000), 对应 100%
/*
 * 应用层占空比上限(不是库里的满量程)
 *
 * 库里的 PWM_DUTY_MAX = 10000 是【分母】, 代表"满量程",
 * 改成别的值会改变所有占空比的语义, 所以库那边必须保持 10000。
 *
 * 这里才是真正限制电机出力的地方: hw_motor_set 会把指令夹在
 * [-MOTOR_DUTY_MAX, +MOTOR_DUTY_MAX] 内。
 *
 * 3000 = 30% 占空比, 空载约 2000 RPM。调车初期有它兜底,
 * 即使 PID 算飞了也不会打满占空比。
 * 跑熟了想提速再往上加, 但注意 10000 = 100% 会直接烧电机。 */
#define MOTOR_DUTY_MAX      (5000)

/*
 * 方向标定, 上电实测后按需改
 * 给 hw_motor_set 正数时, 轮子应该朝车头方向转
 * 哪一侧转反了, 就把对应的宏改成 1, 不用动接线
 */
#define MOTOR_L_INVERT      (0)
#define MOTOR_R_INVERT      (1)

void hw_motor_init(void);

//left/right 取 -MOTOR_DUTY_MAX ~ +MOTOR_DUTY_MAX
//正数前进, 负数后退, 0 刹车
//超出范围会被限幅
void hw_motor_set(int16 left, int16 right);

//两轮刹车
void hw_motor_stop(void);

#endif
