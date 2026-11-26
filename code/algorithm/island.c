#include "island.h"
#include "ctrl.h"              /* CTRL_PERIOD_MS */
#include "hardware/hw_imu.h"
#include <math.h>

/* ============================================================================
 * island.c --- 环岛状态机实现
 *
 * 全部状态都是 static：程序生命周期内只有一辆车、一个环岛，不需要实例化。
 * 状态由 ctrl_tick() 在 TIM7 中断里推进（4ms 一次），
 * 入口 island_set_vision_on() 由主循环 task_track 每帧喂（约 20ms 一次）。
 * 两者频率不同但都是"最新值"语义，用 volatile 就行，不需要队列。
 * ==========================================================================*/

//==================== 内部状态 ====================

static volatile island_state_enum s_st = ISLAND_ST_IDLE;

/* 入环瞬间记录的车头朝向（度）。所有判据都是相对它的差值。 */
static float s_start_yaw = 0.0f;

/* 视觉这一帧报的"在不在环岛里" */
static volatile uint8 s_vision_on = 0;

/* 环岛左右方向，只做遥测 */
static volatile int s_side = 0;

/* 一级判据是否已通过（转过 > ISLAND_ENTER_DEG）。
 *
 * 这是整套逻辑的核心防护：没有它，刚进环 Δyaw ≈ 0 会立刻满足 < 25 而出岛。
 * 一旦置 1 就保持到本次环岛结束。 */
static uint8 s_exit_armed = 0;

/* 计时器，单位 ms，由 island_tick 按 CTRL_PERIOD_MS 累加 */
static uint32 s_running_ms = 0;
static uint32 s_after_ms   = 0;     /* AFTER 阶段已过的时间 */

/* AFTER 阶段：本周期给 ctrl 的 yaw 转向量（RPM 量纲，和视觉 turn 一致） */
static volatile float s_after_turn = 0.0f;

/* AFTER 阶段的积分累加器（单位：度·秒）
 *
 * 每周期 += delta * dt，乘以 ISLAND_AFTER_KI 得到 RPM。
 * 作用：消除纯 P 的稳态误差 —— delta 很小时 P 项推不动车，
 *      但积分会持续累加直到推得动。
 *
 * 每次进入 AFTER 时清零，避免上一次的残留影响这一次。 */
static float s_after_integral = 0.0f;

/* 待处理"请视觉退出"请求 */
static volatile uint8 s_exit_req = 0;

/* 最近一次算出的相对角度，遥测用 */
static volatile float s_delta_yaw = 0.0f;

//==================== 工具 ====================

/* 把角度折到 -180 ~ +180。
 *
 * 为什么每个差值都要折:
 *   陀螺积分出的 yaw 在 0/360 附近会跳变。
 *   比如起点 350°，现在 10°，raw 差值 = -340°，实际只转了 +20°。
 *   不折的话，判据 "|Δ| < 25" 会把 -340 判成"没转回来"，
 *   判据 "|Δ| > 60" 又会误触发。折一次就正常了。
 *
 * 用 fmodf 而不是 if 加减 360:
 *   差值理论上限是 ±360，但万一 yaw 本身有 unwrap 越界，if 只减一次不够。
 *   fmodf 一次到位。
 */
static float wrap180(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f)
    {
        a += 360.0f;
    }
    return a - 180.0f;
}

//==================== 对外接口 ====================

void island_init(void)
{
    s_st         = ISLAND_ST_IDLE;
    s_start_yaw  = 0.0f;
    s_vision_on  = 0;
    s_side       = 0;
    s_exit_armed = 0;
    s_running_ms = 0;
    s_after_ms   = 0;
    s_exit_req   = 0;
    s_delta_yaw  = 0.0f;
    s_after_turn = 0.0f;
    s_after_integral = 0.0f;
}

void island_set_vision_on(int on)
{
    s_vision_on = (on != 0) ? 1u : 0u;
}

void island_set_side(int side)
{
    if (side > 0)      { s_side = 1; }
    else if (side < 0) { s_side = -1; }
    else               { s_side = 0; }
}

uint8 island_take_exit_request(void)
{
    uint8 r = s_exit_req;

    s_exit_req = 0;         /* 读过就清，避免视觉被反复通知 */
    return r;
}

island_state_enum island_get_state(void)
{
    return s_st;
}

float island_get_delta_yaw(void)
{
    return s_delta_yaw;
}

uint8 island_is_exit_armed(void)
{
    return s_exit_armed;
}

/* --------------------------------------------------------------------------
 * 转向混合（ctrl_tick 每 4ms 调一次）
 *
 * 返回 0~1 的权重：
 *   0   = 完全用视觉 PID
 *   1   = 完全用 yaw 锁定
 *   中间 = 交接期的线性混合
 *
 * 只在 AFTER 阶段返回非 0。
 * RUNNING 阶段仍然走视觉 —— 你实测环内视觉效果还行，不动它。
 * -------------------------------------------------------------------------- */
float island_get_turn_blend(float *yaw_turn)
{
    float w = 0.0f;

    if (ISLAND_ST_AFTER == s_st)
    {
        /* AFTER 时间内权重恒为 1（turn 自己在末尾 100ms 内部衰减），
         * 这样视觉完全让位，不会和 yaw 打架。 */
        w = 1.0f;
    }
    else
    {
        s_after_turn = 0.0f;
    }

    if (yaw_turn)
    {
        *yaw_turn = s_after_turn;
    }
    return w;
}

/* --------------------------------------------------------------------------
 * 速度缩放
 *
 * RUNNING 降到 75%：环内视野遮挡重，慢一点视觉稳得住。
 * AFTER   0.85 → 1.0：出岛时略快一点冲出来，最后 100ms 恢复到全速，
 *                     和转向的衰减同步，避免"速度恢复 + 转向突变"同时发生。
 * -------------------------------------------------------------------------- */
float island_get_speed_scale(void)
{
    if (ISLAND_ST_RUNNING == s_st)
    {
        return ISLAND_SPEED_RUNNING;
    }

    if (ISLAND_ST_AFTER == s_st)
    {
        /* 和 turn 衰减用同一套比例，两者同时归位。
         *
         * 注意 s_after_ms 可能已经 >= ISLAND_AFTER_MS（island_tick 和
         * 本函数在同一周期内的调用顺序不固定），必须先判上界再减，
         * 否则无符号下溢。 */
        if (s_after_ms < ISLAND_AFTER_MS)
        {
            uint32 left_ms = ISLAND_AFTER_MS - s_after_ms;

            if (left_ms < ISLAND_AFTER_BLEND_MS)
            {
                return ISLAND_SPEED_AFTER +
                       (1.0f - ISLAND_SPEED_AFTER) *
                       (1.0f - (float)left_ms / (float)ISLAND_AFTER_BLEND_MS);
            }
        }
        return ISLAND_SPEED_AFTER;
    }

    return 1.0f;
}

/* --------------------------------------------------------------------------
 * 状态机主体。每 CTRL_PERIOD_MS 调一次。
 * -------------------------------------------------------------------------- */
void island_tick(void)
{
    imu_t ang;
    uint8 have_yaw;
    float delta;

    /* ---- 先读 yaw。读不到就什么都不做，保持当前状态 ----
     *
     * 为什么不在读不到时退出:
     *   软件 I2C 偶发丢一帧很常见（主循环被视觉抢走几个 ms）。
     *   一次读失败就退出环岛状态，车会在环里突然交回视觉巡线、甩出去。
     *   保留状态，下一周期再读。真长时间坏掉由 ISLAND_TIMEOUT_MS 兜底。 */
    have_yaw = (hw_imu_get_angle(&ang) == 0);

    switch (s_st)
    {
        //==================================================================
        case ISLAND_ST_IDLE:
        //==================================================================
        {
            s_running_ms = 0;
            s_after_ms   = 0;
            s_exit_armed = 0;
            s_delta_yaw  = 0.0f;
            s_after_turn = 0.0f;
            s_after_integral = 0.0f;

            if (!s_vision_on)
            {
                break;                  /* 视觉还没说进环，什么都不做 */
            }

            if (!have_yaw)
            {
                break;                  /* 视觉说进环了，但读不到 yaw。
                                         * 不记起点 —— 记 0 会让后面全错。
                                         * 等下一周期读到再记。 */
            }

            /* ---- 记起点，进 RUNNING ---- */
            s_start_yaw  = ang.yaw;
            s_delta_yaw  = 0.0f;
            s_exit_armed = 0;
            s_running_ms = 0;
            s_st         = ISLAND_ST_RUNNING;
            break;
        }

        //==================================================================
        case ISLAND_ST_RUNNING:
        //==================================================================
        {
            if (!have_yaw)
            {
                /* 同上：读不到就保持，不动判据 */
                break;
            }

            /* 相对入环起点的已转角度 */
            delta = wrap180(ang.yaw - s_start_yaw);
            s_delta_yaw = delta;

            if (s_running_ms < 0xFFFFFFFFu)
            {
                s_running_ms += CTRL_PERIOD_MS;
            }

            /* ---- 一级判据：真的转进去了 ---- */
            if (!s_exit_armed)
            {
                if (fabsf(delta) > ISLAND_ENTER_DEG)
                {
                    s_exit_armed = 1;
                }
                /* 一级没过之前，无论 delta 多小都不判出岛。
                 * 这段就是"刚进环 Δyaw≈0"的误判窗口，必须跳过。 */
                break;
            }

            /* ---- 二级判据：转回来了 ---- */
            if (fabsf(delta) < ISLAND_EXIT_DEG)
            {
                s_st       = ISLAND_ST_AFTER;
                s_after_ms = 0;
                s_exit_req = 1;       /* 通知视觉清 g_island_on */
                break;
            }

            /* ---- 三级：超时兜底 ---- */
            if (s_running_ms > ISLAND_TIMEOUT_MS)
            {
                s_st       = ISLAND_ST_AFTER;
                s_after_ms = 0;
                s_exit_req = 1;
            }
            break;
        }

        //==================================================================
        case ISLAND_ST_AFTER:
        //==================================================================
        {
            /* ------------------------------------------------------------------
             * 出岛动作：yaw 锁朝向，不看视觉
             *
             * 学学长 CMD_AFTER（moto.c:340-347）：
             *   出岛后用 normalize(yaw - start) 做角度环，把车头拉回
             *   入环前的朝向，直行穿出出口区域。150 个控制周期不看视觉。
             *
             * 为什么必须这么做：
             *   出岛那一刻视觉看到的是环岛出口的弧线，err 指向弧线切线
             *   方向而不是直道方向。直接交回视觉，车会跟着弧线甩出去。
             *   用 yaw 锁住"入环前的朝向"，出口必然是对的。
             *
             * 为什么 delta 要重新算而不是用 RUNNING 里的：
             *   出岛判定是 |delta| < 25，此刻 delta 可能还有 20 度残余。
             *   AFTER 阶段就是要把这 20 度收掉，所以每个周期都要重算。
             * ------------------------------------------------------------------ */

            if (s_after_ms < 0xFFFFFFFFu)
            {
                s_after_ms += CTRL_PERIOD_MS;
            }

            /* 读不到 yaw 就直接结束 AFTER，交回视觉。
             * 硬撑着用上一帧的 delta 继续打角，可能把车拐进沟里。 */
            if (!have_yaw)
            {
                s_st         = ISLAND_ST_IDLE;
                s_exit_armed = 0;
                s_delta_yaw  = 0.0f;
                s_after_turn = 0.0f;
                s_after_integral = 0.0f;
                break;
            }

            delta = wrap180(ang.yaw - s_start_yaw);
            s_delta_yaw = delta;

            /* ---- 积分项累加 ----
             *
             * 累加的是 delta（度）乘以 dt（秒），
             * 得到"度·秒"，乘以 KI 后和 KP*delta 量纲一致（都是 RPM）。
             *
             * 防饱和：限制在 +-ISLAND_AFTER_INT_MAX。
             * 不限制的话，如果 delta 长时间偏一个方向（比如车卡住），
             * 积分会涨到天文数字，AFTER 结束后突然释放，把车甩出去。
             */
            {
                float dt = (float)CTRL_PERIOD_MS * 0.001f;

                s_after_integral += delta * dt;

                if (s_after_integral >  ISLAND_AFTER_INT_MAX) {
                    s_after_integral =  ISLAND_AFTER_INT_MAX;
                }
                if (s_after_integral < -ISLAND_AFTER_INT_MAX) {
                    s_after_integral = -ISLAND_AFTER_INT_MAX;
                }
            }

            /* ---- yaw -> turn: P + I ----
             *
             * P 项: 快速响应大偏差（delta = 20 度 -> 700 RPM）
             * I 项: 慢慢消除小偏差（delta = 3 度时 P 只有 105 RPM，
             *       但积分会持续累加直到够用）
             *
             * 符号见 island.h 的 ISLAND_YAW_TURN_SIGN 注释。
             * 积分项用同样的符号，方向才一致。 */
            s_after_turn = ISLAND_YAW_TURN_SIGN *
                           (ISLAND_AFTER_KP * delta +
                            ISLAND_AFTER_KI * s_after_integral);

            /* 时间到，交回视觉。
             * 放在衰减之前：s_after_ms >= ISLAND_AFTER_MS 时
             * ISLAND_AFTER_MS - s_after_ms 会无符号下溢成天文数字。 */
            if (s_after_ms >= ISLAND_AFTER_MS)
            {
                s_st         = ISLAND_ST_IDLE;
                s_exit_armed = 0;
                s_delta_yaw  = 0.0f;
                s_after_turn = 0.0f;
                s_after_integral = 0.0f;
                break;
            }

            /* 末尾衰减：让 turn 平滑归零，视觉平滑接管。
             * 不衰减的话，交接瞬间 turn 会从 yaw 值突跳到视觉 PID 值。
             *
             * 这里 s_after_ms < ISLAND_AFTER_MS 已由上面保证，
             * 减法不会下溢。 */
            {
                uint32 left_ms = ISLAND_AFTER_MS - s_after_ms;

                if (left_ms < ISLAND_AFTER_BLEND_MS)
                {
                    s_after_turn *= (float)left_ms / (float)ISLAND_AFTER_BLEND_MS;
                }
            }
            break;
        }

        //==================================================================
        default:
        //==================================================================
        {
            s_st = ISLAND_ST_IDLE;
            break;
        }
    }
}
