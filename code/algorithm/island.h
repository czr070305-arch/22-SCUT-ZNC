#ifndef _ISLAND_H_
#define _ISLAND_H_

#include "zf_common_headfile.h"

/* ============================================================================
 * island.h --- 环岛状态机（控制层，用陀螺仪判出岛）
 *
 * ---------------------------------------------------------------------------
 * 职责边界（这是本模块存在的全部理由）
 *
 *   视觉层（vision_element.c）：  识别环岛入口、判左/右、发"该进环了"
 *   控制层（本模块）：            记 yaw 起点、判"转满一圈"、通知视觉退出
 *
 *   视觉管"进"，控制管"出"。
 *   因为"转了多远"是惯性量，图像看不见自己绕了多久。
 */

//==================== 判据门限（度） ====================
/* 一级：确认"真的进环了"需要转过的角度。*/
#define ISLAND_ENTER_DEG        (60.0f)

/* 二级：出岛判据。车头回到入环前的朝向 ±25° 以内算转满一圈。*/
#define ISLAND_EXIT_DEG         (40.0f)

/* 三级：控制层超时兜底（毫秒）。 */
#define ISLAND_TIMEOUT_MS       (12000u)

/* --------------------------------------------------------------------------
 * 出岛动作（ALTERED 2026-11-26：学学长 CMD_AFTER）
 * --------------------------------------------------------------------------
 * 问题：出岛瞬间如果立刻交回视觉，视觉看到的是环岛出口的弧线，
 *       err 指向弧线方向，车会跟着甩出去。
 *
 * 学长的做法（moto.c:340-347 CMD_AFTER）：
 *   出岛后用 yaw 闭环把车头拉回入环前的朝向，直行穿出出口区域，
 *   完全不看视觉。150 个控制周期后才交回。
 *
 * 我们照做，但加两处改进：
 *   1) 末尾 100ms 线性衰减，避免交回视觉时 turn 从满值跳到视觉值
 *   2) 整个环岛阶段降速，不只出岛
 * -------------------------------------------------------------------------- */

/* AFTER 阶段持续时间（毫秒）。
 * 要够车穿过出口区域（约 0.3~0.5 米）。
 * 学长用 150 个 1kHz 周期 = 150ms，我们用 250ms 更稳。 */
#define ISLAND_AFTER_MS         (800u)

/* AFTER 阶段末尾与视觉交接的衰减时长（毫秒）。
 * 这段时间里 yaw 转向量线性衰减到 0，视觉 err 平滑接管。
 * 必须 < ISLAND_AFTER_MS。 */
#define ISLAND_AFTER_BLEND_MS   (150u)

/* yaw 锁定的比例增益（RPM / 度）。*/
#define ISLAND_AFTER_KP         (20.0f)

/* ---- AFTER 阶段的积分项（2026-11-27 新增）----
 *
 * 解决纯 P 的稳态误差。
 *
 * 为什么纯 P 不够:
 *   turn = KP * delta，delta 越小 turn 越小。
 *   delta = 5 度 -> turn = 175 RPM -> 差速比只有 8%
 *   对应转弯半径 1.8 米，环岛半径才 0.25 米。
 *   结果就是 delta 卡在 3~8 度再也降不下去，
 *   车带着这个倾角冲出环岛，出口处车头偏内。
 *
 * 积分的作用:
 *   即使 delta 一直很小，积分也随时间持续累加，
 *   最终能产生足够的 turn 把车推到 0 度。
 *   这就是 P 和 PI 的本质区别。
 *
 * 单位: RPM / (度 * 秒)
 *   积分量 = sum(delta * dt)，单位是 度*秒
 *   乘以 KI 得到 RPM，和 KP * delta 量纲一致
 *
 * 取值推算:
 *   目标: delta = 5 度时，0.4 秒内积累出 300~400 RPM 的修正力
 *   积分量 = 5 * 0.4 = 2.0 度*秒
 *   需要 KI = 350 / 2.0 = 175
 *   取 150 保守一点
 */
#define ISLAND_AFTER_KI         (0.0f)

/* 积分上限（度*秒），防止长时间偏置导致积分饱和。
 *
 * 150 * 5 = 750 RPM 是积分单独能贡献的最大值，
 * 加上 P 项后容易被 turn_lim(1500) 截断，所以不要太大。
 * 太小则防不住稳态误差。5 是折中。 */
#define ISLAND_AFTER_INT_MAX    (5.0f)

/* yaw 方向 -> turn 符号的对应关系。
 *   1) yaw 方向  见 task_imu.h 第 16 行:  (右- 左+)
 *      车头往左转 -> yaw 增大
 *      车头往右转 -> yaw 减小
 *
 *   2) turn 方向  见 ctrl.c:  left = base - turn, right = base + turn
 *      turn > 0 -> 左轮慢右轮快 -> 车头向左
 *      turn < 0 -> 左轮快右轮慢 -> 车头向右
 */
#define ISLAND_YAW_TURN_SIGN    (-1.0f)

/* 环岛各阶段的速度缩放（1.0 = 不缩放）。
 * 学长：cmd_go_circle != NONE 时降到 1500/2000 = 75%。
 * 我们分两档：环内慢一点看得清线，出岛稍快一点冲出去。 */
#define ISLAND_SPEED_RUNNING    (1.0f)
#define ISLAND_SPEED_AFTER      (1.4f)

//==================== 接口 ====================

/* 初始化。ctrl_init() 里调用一次。 */
void island_init(void);

/* 每 CTRL_PERIOD_MS 调用一次，由 ctrl_tick() 调用。
 * 不读图像、不调 IMU 时的代价：一次状态判断，几微秒。 */
void island_tick(void);

/* 视觉这一帧是否在环岛里。task_track.c 每帧喂进来。
 * 只在 IDLE 状态下起作用（用来记起点）；进环后本模块自己管退出。 */
void island_set_vision_on(int on);

/* 视觉这一帧报的环岛左右方向（-1 左 / +1 右），用于遥测。
 * 传 0 表示不关心。 */
void island_set_side(int side);

/* 1 = 已经判出岛，视觉应该清掉 g_island_on 了。
 * task_track.c 读到 1 就调 vision_island_exit()，然后本函数自动清位。 */
uint8 island_take_exit_request(void);

/* 当前状态（遥测/屏幕用） */
typedef enum
{
    ISLAND_ST_IDLE    = 0,    /* 不在环岛 */
    ISLAND_ST_RUNNING = 1,    /* 环内，视觉巡线 */
    ISLAND_ST_AFTER   = 2,    /* 出岛中，yaw 锁朝向（学学长 CMD_AFTER） */
} island_state_enum;

island_state_enum island_get_state(void);

/* 相对入环起点的已转角度（度，-180~+180），遥测用。
 * 不在环岛时返回 0。 */
float island_get_delta_yaw(void);

/* 出岛判据是否已满足（一级已过 且 二级满足）。遥测/调试用。 */
uint8 island_is_exit_armed(void);

/* ==========================================================================
 * 转向覆盖（ctrl_tick 每 4ms 调）
 * ==========================================================================
 *
 * 返回 -1.0 ~ +1.0 的权重 w：
 *   w > 0 : 本周期应该【以 w 的比例】用 yaw 锁定转向替代视觉 PID
 *   w == 0: 正常用视觉 PID
 *
 * 实际用法（ctrl.c 里）：
 *     w = island_get_turn_blend(&yaw_turn);
 *     turn_pid  = PID_Calc(...);                      // 视觉
 *     turn = turn_pid * (1 - w) + yaw_turn * w;       // 混合
 *
 * 为什么返回权重而不是直接返回 turn：
 *   AFTER 末尾 100ms 要让 yaw 转向平滑衰减到 0、视觉平滑接管。
 *   如果只返回 turn，交接那一刻会从"yaw 满值"突跳到"视觉值"，
 *   车会抖一下。混合权重让这个过渡连续。
 *
 * 只在 ISLAND_ST_AFTER 状态返回非 0。
 */
float island_get_turn_blend(float *yaw_turn);

/* 速度缩放系数，ctrl_tick 里乘到 base 上。
 *   1.0        = 不缩放（不在环岛）
 *   0.75       = 环内（RUNNING）
 *   0.85 -> 1.0= 出岛（AFTER，最后 100ms 线性恢复）
 *
 * 为什么要降速：
 *   环内视野遮挡严重，速度低一点视觉更容易稳住；
 *   出岛 yaw 锁朝向时也要低速，否则 P 控制来不及收敛。
 *   学长同样做法（1500/2000 = 75%）。 */
float island_get_speed_scale(void);

#endif /* _ISLAND_H_ */
