#ifndef _CTRL_H_
#define _CTRL_H_

#include "zf_common_headfile.h"
#include "tune.h"

/*
 * 循迹控制层
 *
 * 职责: 把视觉给的一维误差 err 变成左右轮的目标转速
 *
 * ---------------------------------------------------------------
 * 串级结构
 *
 *   视觉 err  -->  [转向环 位置式P]  -->  turn (差速 RPM)
 *                                            |
 *                          base(巡航速度) +/- turn
 *                                            |
 *                                            v
 *                              [左右轮速度环 增量式PID]  --> 占空比
 *                                            |
 *                                            v
 *                                        8701 -> 电机
 *
 * 转向环是外环, 速度环是内环. 速度环在 chassis 里, 本模块只到"目标 RPM".
 * ---------------------------------------------------------------
 *
 * 符号约定 (非常重要, 接反了车会往反方向打)
 *
 *   err > 0  线在画面右侧  -->  车应该右转
 *   右转 = 左轮快、右轮慢  (左轮在车体左侧, 转得快就会把车头往右推)
 *   所以: left = base - turn, right = base + turn
 *   要满足 err > 0 时 turn < 0, 即 turn 与 err 反号
 *
 *   本模块用标准写法 PID_Calc(target=0, feedback=err):
 *     内部 e = 0 - err = -err
 *     err > 0 -> e < 0 -> turn < 0 -> left 变大 -> 右转  正确
 *
 *   改这段之前先画一遍: 左轮快=车头向右. 这是差速车最反直觉的一点.
 * ---------------------------------------------------------------
 *
 * 目前视觉还没移植过来
 *
 *   ctrl_tick() 读的是 ctrl_set_error() 写进来的值.
 *   暂时没有视觉时, 该值恒为 0, 行为就是"两轮同速直行".
 *   等 vision.c 写好, 在 task_track_run() 里把 g_vis.err 喂进
 *   ctrl_set_error() 即可, 本文件不用改.
 * ---------------------------------------------------------------
 */

/* ==================== 可调参数 ==================== */

/* ==================== 巡航速度 ====================
 *
 * 这两个值已经挪到 tune 模块：TRACK 页可调，Flash 保存。
 *   当前生效 = g_tune[TUNE_CRUISE]
 *   上电初值 = g_tune_def[TUNE_CRUISE]（第一次上电 = 出厂值）
 *
 * 下面只留范围上下限，供 ctrl_set_cruise() 做保护性钳位。
 * 出厂值本体在 tune.c 的 g_tune_items[] 里，只有那一份。
 */
#define CTRL_CRUISE_MIN         (600.0f)
#define CTRL_CRUISE_MAX         (3000.0f)

/* 内轮死区 (RPM)  -- 实测值
 * 低于这个转速电机不转. 用于 base 的下限计算, 不用于差速限制
 * (差速限制见 CTRL_INNER_WHEEL_MIN_RPM). */
#define CTRL_WHEEL_MIN_RPM      (520.0f)

/* 最小差速预算 (RPM)
 * base 的下限 = WHEEL_MIN + TURN_MIN. 防止元素降速把 base 压到
 * 电机都带不动、或者转向空间全被吃掉. */
#define CTRL_TURN_MIN_RPM       (160.0f)

/* 转向时内轮的最低转速 (RPM)  -- 决定最小转弯半径
 *   0 = 允许内轮完全停转 (推荐, 直角弯需要)
 * 转弯半径 R = 轮距 x base / (2 x turn), turn 上限 = base - 这个值.
 *
 * 2026-11-19 实车: 设成 250 时直角弯过不去. base=680 时 turn 上限只剩
 * 430, R = 0.79 x 轮距, 内轮一直 >=250 把车往前推, 永远绕不进弯.
 * 改回 0: turn 上限 = 680, 内轮可停, R = 0.50 x 轮距.
 * 实测两轮 1500 RPM 都跟得住(MOTOR 页), 转向权限不该被这里卡死. */
#define CTRL_INNER_WHEEL_MIN_RPM    (0.0f)

/* ==================== 转向 P（tune 模块） ====================
 *
 * 当前生效值来自 tune：
 *   CTRL_TURN_KP_LO  ->  g_tune[TUNE_KP_LO]   屏幕显示 KP_LN（直线 P）
 *   CTRL_TURN_KP_HI  ->  g_tune[TUNE_KP_HI]   屏幕显示 KP_CR（弯道 P）
 *
 * 分段插值的理由见 ctrl.c 的 ctrl_turn_kp() 注释。
 * ERR_LO / ERR_HI 是插值过渡区，调整频率极低，仍保留为宏。
 *
 * KI / KD 恒为 0：
 *   视觉 err 已经有斜率项（CFG_ERR_SLOPE_W）提供"预判"，
 *   再加 D 相当于二次求导，直线噪声会被剧烈放大。
 */
#define CTRL_TURN_KP_INIT       (2.5f)      /* 仅 PID_Init 的占位值 */

/* 分段插值的过渡区（err 单位 = 像素）
 * |err| < ERR_LO   全用 KP_LN
 * |err| > ERR_HI   全用 KP_CR
 * 中间线性插值。这两个值是"结构参数"，不在屏上调。 */
#define CTRL_TURN_ERR_LO        (12.0f)
#define CTRL_TURN_ERR_HI        (60.0f)

/* 转向输出上限（RPM）  -->  tune 的 TUNE_TURN_MAX（屏幕 TMAX）
 *
 * 实际生效的是 turn_lim = min(base - INNER_WHEEL_MIN, TUNE_TURN_MAX)。
 * 弯道半径不够就加大 TMAX；直线不需要它（直线靠 KP_LN 压低）。
 */

/* 转向死区 (err 单位)
 * 视觉 err 有 1~2px 抖动, 死区取 2.5 避免直道上无意义的来回修正 */
#define CTRL_TURN_DEAD_ZONE     (1.5f)   /* 2026-11-24: 2.5->1.5，小偏差也要修 */

/* 转向环积分限幅
 * Ki=0 时不影响输出, 但 PID_Calc 内部仍累加 err_sum.
 * 限幅防止以后开 I 时积分饱和. */
#define CTRL_TURN_INT_MAX       (50.0f)

/* 弯道减速下限  -->  tune 的 TUNE_CURVE_SLOW（屏幕 SLOW）
 *
 * 存储值是 50~100（整数，代表 0.50~1.00），tune_get() 自动除以 100。
 * 弯道冲出去就调小；弯道速度不够就调大。
 */
#define CTRL_CURVE_ERR_SPAN     (80.0f)

/* 丢线保护: 三阶段
 *   阶段 1  HOLD     0 ~ 300ms    保持上一拍转向, 速度 x0.75
 *   阶段 2  RECOVER  300 ~ 600ms  转向回正, 速度 x0.75
 *   阶段 3  STOP     600ms+       刹停
 *
 * 为什么要阶段 1: 视觉 50fps, 单帧丢线很常见(反光/阴影).
 * 一丢线就回正会让方向盘抽一下; 保持上一拍转向最稳.
 *
 * 2026-11-19 实车: HOLD 只有 100ms 太短 —— 直角弯里线一断, 100ms 后
 * RECOVER 就 turn=0 强制回正, 等于弯道中途主动放弃转向(正是"弯不过去"
 * 的第二种成因). 加长到 300ms, 让车带着弯道转向撑过短暂断线.
 *
 * STOP 1500ms 太长: 按 800RPM 估算要直行 1.4s / 近半米才刹车, 丢线时
 * 车会一路冲出赛道. 收回 600ms.
 *
 * 2026-11-19 直线抽搐根因:
 *   单帧丢线 -> base = 800 x 0.75 = 600 -> 被 base_floor(680) 抬回 680
 *   -> 恢复后回到 800, 左右轮目标两拍内跳 120 RPM.
 *   增量式速度环 Kp=18.2 => 单拍占空比变化 18.2 x 120 ~ 2184/3000,
 *   电机猛抽一下, 发生时刻由视觉随机丢帧决定, 所以"走着走着抽一下".
 *
 *   解法: 丢线时不再降速(SLOW 系数置 1.0), 目标连续, 阶跃消失.
 *   安全保护仍然在: 600ms 后照常刹停.
 *
 * HOLD 阶段不调 PID_Calc: 视觉丢线时 err 是旧值且不变,
 * 反复喂同一个 err 会让积分持续累加(windup). */
#define CTRL_LOST_HOLD_MS       (300)
#define CTRL_LOST_HOLD_SLOW     (1.0f)
#define CTRL_LOST_STOP_MS       (600)
#define CTRL_LOST_SLOW_RATIO    (1.0f)

/* 起步斜坡 (ms)
 * 直接给满速目标会让车"蹭"一下窜出去. 从死区速度爬到目标速度. */
#define CTRL_RAMP_MS            (300)

/* 控制周期, 必须和 chassis 的 CHASSIS_PERIOD_MS 一致
 * (hw_encoder.h 里 ENC_SAMPLE_MS = 20) */
#define CTRL_PERIOD_MS          (4)   /* 2026-11-24: 20->4，TIM7 实际 4ms 中断，原来写 20 导致丢线保护快5倍 */

/* ==================== 运行状态 ==================== */

typedef enum
{
    CTRL_STATE_IDLE = 0,        //未启动, 底盘保持停车
    CTRL_STATE_RUN,             //正常循迹
    CTRL_STATE_LOST,            //丢线降速中
    CTRL_STATE_STOPPED,         //丢线超时, 已刹车
}ctrl_state_enum;

/* ==================== 接口 ==================== */

//初始化: 建 PID + 清状态. 必须在 chassis_init 之后调用
void ctrl_init(void);

/*
 * 喂入本帧视觉误差. 主循环里调用, 频率约等于视觉帧率.
 * 内部只存值, 不做控制计算.
 *   参数 err: 视觉的 g_vis.err. 负=线偏左, 正=线偏右
 *   参数 valid: 1=本帧视觉可信, 0=不可信(丢线/置信度低)
 */
void ctrl_set_error(float err, uint8 valid);

//启动循迹 (软启动斜坡生效). 置启动请求, 下一控制拍在 TIM7 里生效
void ctrl_start(void);

/*
 * 在线改巡航速度 (RPM)
 * 运行中也可以改, 下一控制周期生效.
 * 传进来的值会被夹在 [CTRL_CRUISE_MIN, CTRL_CRUISE_MAX] 内.
 */
void ctrl_set_cruise(float rpm);

//当前巡航速度 (RPM)
float ctrl_get_cruise(void);

/*
 * 速度提示系数 (0.0 ~ 1.0), 乘在巡航速度上
 *
 * 用途: 视觉判定环岛 / 岔路时压速度, 但控制层不需要知道"为什么".
 *       这样 ctrl.c 不依赖 vision.h, 换视觉算法不用改控制代码.
 *
 *   1.0 = 不额外减速 (默认)
 *   0.6 = 环岛
 *   0.7 = 岔路
 *
 * 每个控制拍生效, 不调用则保持上次的值. ctrl_start() 时复位为 1.0.
 */

/* 停止: 置停车请求, 下一控制拍(<=20ms)在 TIM7 里执行刹车并回到 IDLE.
 * 不在调用处直接刹车, 避免和正在跑的 chassis_tick 抢电机/PID 状态. */
void ctrl_stop(void);

/*
 * 控制步进. 每 CTRL_PERIOD_MS 调一次.
 *
 * 调用方式: 在 TIM7 中断里调用, 且必须排在 chassis_tick() 之前
 * (见 isr.c). 同一拍内: ctrl_tick 先算出新目标并下发, chassis_tick
 * 再用新目标跑速度环, 目标生效延迟为 0.
 *
 * 为什么必须放中断: 主循环一圈 8~25ms 不等(视觉 6~9ms + 刷屏 10~13ms),
 * 用 chassis_get_tick() 轮询触发会让控制节拍随机抖动, 偶尔丢一拍。
 * ctrl_tick 只有几十条浮点运算(<50us), 对 TIM7 无负担。
 */
void ctrl_tick(void);

//当前状态
ctrl_state_enum ctrl_get_state(void);

//上一周期的实际输出, 给屏显用
void ctrl_get_output(float *left_rpm, float *right_rpm, float *turn);

//本拍转向分量 (RPM), 便于 VOFA 单通道看曲线
float ctrl_get_turn(void);

//最近一次喂进来的 err (调试用)
float ctrl_get_error(void);

#endif
