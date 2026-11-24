#ifndef _HW_ENCODER_H_
#define _HW_ENCODER_H_

#include "zf_common_headfile.h"

/*
 * MT6701CT-ACD 磁编码器驱动, ABZ 增量输出
 *
 * 型号含义(数据手册第5页): AB=1024脉冲/圈, Z=4LSB, 逆时针角度增加
 *
 * 编码器板接线要点:
 *   只焊 R-ABZ(选ABZ模式), R-IIC-1/R-IIC-2 不能焊, 否则 3.3V 短地
 *   XH2.54-4P 座子焊在 2~5 脚
 *   磁铁: 6mm 直径, 2.5mm 厚, 径向充磁, 气隙 0.5~2mm, 偏心 <0.3mm
 *
 * 逐飞 encoder_quad_init 用 TIM_EncoderMode_TI2 + 双上升沿:
 *   第二参数(ch1)只判方向, 第三参数(ch2)每个上升沿计一次数
 *   所以每圈计数 = 1024(不是4096)
 *
 * 定时器占用: TIM5(左), TIM4(右)
 */

//==================== 引脚与定时器 ====================
//左编码器: 编码器板 B -> PA0, A -> PA1
#define ENC_L_TIM           (TIM5_ENCOEDER)
#define ENC_L_CH1           (TIM5_ENCOEDER_MAP0_CH2_A1)
#define ENC_L_CH2           (TIM5_ENCOEDER_MAP0_CH1_A0)

//右编码器: 编码器板 B -> PD12, A -> PD13
#define ENC_R_TIM           (TIM4_ENCOEDER)
#define ENC_R_CH1           (TIM4_ENCOEDER_MAP1_CH2_D13)
#define ENC_R_CH2           (TIM4_ENCOEDER_MAP1_CH1_D12)

//==================== 换算参数 ====================
//编码器每圈计数(MT6701CT-ACD 出厂 1024 线)
#define ENC_COUNT_PER_REV   (1024)

//速度采样周期(ms), 同时也是控制周期
//取 20 而不是 10: 370 电机没有减速箱, 换向器纹波很明显。
//10ms 窗口在 550RPM 时只能覆盖 0.46 个换向周期, 抓到的都是瞬时片段,
//读数会上下跳 ±5%, 被 Kp 放大后变成占空比抖动。
//改到 20ms 后窗口覆盖 0.92 个换向周期, 噪声大约降到 1/3。
//
//注意改这里会连带影响:
//   1. CHASSIS_PERIOD_MS 自动跟着变, PID 每 20ms 算一次
//   2. 转速分辨率从 5.86 提高到 2.93 RPM/计数
//   3. Ki 的等效增益减半, 调参时 Ki 要相应放大一倍
#define ENC_SAMPLE_MS       (4)

//==================== 方向标定 ====================
/*
 * 编码器计数方向标定
 * 目标是: 小车前进时, get_speed() 返回正数
 * 先实测: 让车前进, 看速度是正还是负
 * 哪一侧为负, 就把对应宏改成 1
 */
#define ENC_L_INVERT        (1)
#define ENC_R_INVERT        (0)     

//左右编号
#define ENC_LEFT            (0)
#define ENC_RIGHT           (1)

//==================== 里程标定 ====================
/*
 * 推车标定法(不需要知道减速比和轮径):
 *
 *   1. 进 MOTOR TEST 页, 记下 L CNT 的当前值
 *   2. 把车在桌面上直线推 1000mm(1 米), 越直越准
 *   3. 再记一次 L CNT, 两次的差值记作 N
 *   4. 算 1000000 / N, 填到下面这个宏
 *
 * 例: 推 1 米涨了 4200 个计数
 *     1000000 / 4200 = 238   -> 每个计数 0.238mm
 *
 * 为什么不用"减速比 x 轮径"算:
 *   轮胎实际直径、打滑、磨损都会引入误差, 而且还得去查机械参数。
 *   推车标定把这些一起算进去了, 反而更准。
 *
 * 单位: 0.001 mm/计数。填 0 表示未标定, 此时里程显示会被跳过。
 */
#define ENC_MM_X1000_PER_CNT    (0)

//==================== 接口 ====================
void  hw_encoder_init(void);

//周期采样, 每 ENC_SAMPLE_MS 调一次
//调用者负责保证周期稳定, 否则 RPM 换算会失真
void  hw_encoder_update(void);

//实际转速, 单位 0.01 RPM(即 5860 表示 58.60 RPM)
//用整数返回是为了避免浮点, 显示时除以 100 即可
int32 hw_encoder_get_rpm_x100(uint8 side);

//累计计数, 用于里程
int32 hw_encoder_get_total(uint8 side);

//清计数, 出发前调用
void  hw_encoder_reset(void);

//累计里程, 单位 mm
//标定系数为 0 时恒返回 0
int32 hw_encoder_get_distance_mm(uint8 side);

//当前线速度, 单位 mm/s
//标定系数为 0 时恒返回 0
int32 hw_encoder_get_speed_mm_s(uint8 side);

#endif
