#include "ctrl.h"
#include "pid.h"
#include "chassis.h"
#include "island.h"

/*
 * 符号约定速查 (改代码前先看 ctrl.h 顶部那段)
 *
 *   err > 0 (线在右) -> 需要右转 -> 左轮快右轮慢
 *   本文件用 PID_Calc(target=0, feedback=err), 内部 e = -err
 *   所以 err > 0 时 turn < 0, 而 left = base - turn 就会变大
 */

//转向环: 位置式, 输出是差速量(RPM)
static PID_t ctrl_turn_pid;

/* ---- 主循环 <-> TIM7 共享数据 ----
 *
 * 从本版起 ctrl_tick() 在 TIM7 中断里跑(和速度环同一拍), 下面这些变量
 * 一部分由主循环写(命令/输入), 一部分由中断写(状态/输出), 必须标 volatile,
 * 防止编译器把"只在中断里被改"的变量当死代码优化掉。
 *
 * 原子性说明:
 *   - 单精度 float 在 RV32 上是对齐 32 位单次读写, 硬件层面原子, 无需加锁
 *   - 成对字段(err + valid)用临界区保护, 防止中断读到"新 err + 旧 valid"
 *   - 启动/停车请求用位标志: 主循环只置位, 真正的状态切换留给中断做,
 *     避免主循环在 PID 计算中途改 PID 历史 / 底盘状态
 */
#define CTRL_REQ_START          (1u << 0)
#define CTRL_REQ_STOP           (1u << 1)
static volatile uint8 ctrl_req = 0;

//当前巡航速度(RPM), 可由页面在线改
static volatile float ctrl_cruise   = 800.0f;      /* 占位，ctrl_init 从 tune 取 */

//最近一次视觉给的误差, 以及它是否可信
static volatile float ctrl_err      = 0.0f;
static volatile uint8 ctrl_err_valid = 0;

//运行状态
static volatile ctrl_state_enum ctrl_state = CTRL_STATE_IDLE;

//速度提示系数: 视觉判环岛/岔路时压速度, 1.0 = 不减速

//丢线累计时间(ms)
static uint32 ctrl_lost_ms = 0;

//软启动已运行时间(ms), 到 CTRL_RAMP_MS 为止
static uint32 ctrl_ramp_ms = 0;

//上一周期的输出(中断写, 主循环屏显读)
static volatile float ctrl_out_left  = 0.0f;
static volatile float ctrl_out_right = 0.0f;
static volatile float ctrl_out_turn  = 0.0f;

//初始化
void ctrl_init(void)
{
    //位置式 + 输出限幅 + 积分限幅 + 死区
    //
    //int_max 必须给一个有限值, 不能给 0(0=不限幅).
    //原因: PID_Calc 里 err_sum 是无条件累加的, 现在 Ki=0 所以不影响输出,
    //      但将来为了消除直道偏置打开 Ki 时, err_sum 已经累到几十万,
    //      一启 I 项就会瞬间打出一个巨大的转向量, 车会直接甩出去.
    //      提前限住, 以后开 I 才安全.
    /* 环岛状态机复位。放在 PID_Init 之前，
     * 保证任何控制参数变动都不会留下半个环岛状态。 */
    island_init();

    PID_Init(&ctrl_turn_pid,
             CTRL_TURN_KP_INIT, 0.0f, 0.0f,
             (float)g_tune[TUNE_TURN_MAX], CTRL_TURN_INT_MAX, CTRL_TURN_DEAD_ZONE,
             PID_MODE_POS);

    ctrl_req        = 0;
    ctrl_cruise     = tune_get(TUNE_CRUISE);
    ctrl_err        = 0.0f;
    ctrl_err_valid  = 0;
    ctrl_state      = CTRL_STATE_IDLE;
    ctrl_lost_ms    = 0;
    ctrl_ramp_ms    = 0;
    ctrl_out_left   = 0.0f;
    ctrl_out_right  = 0.0f;
    ctrl_out_turn   = 0.0f;
}

//喂入视觉误差
void ctrl_set_error(float err, uint8 valid)
{
    /* NaN / Inf 兜底. 视觉侧已经清洗过, 这里再挡一次:
     * NaN 与任何数比较都为 false, 会绕过 ctrl_tick 里所有限幅,
     * 直接传进 PID_Calc -> chassis_set_target -> 电机乱转.
     * 一旦收到脏值就当作"本帧无效", 让丢线保护接管. */
    if(!(err == err))
    {
        uint32 primask = interrupt_global_disable();
        ctrl_err_valid = 0;
        interrupt_global_enable(primask);
        return;                     /* 保留上次的 ctrl_err 不动 */
    }
    if(err >  1.0e6f) err =  1.0e6f;
    if(err < -1.0e6f) err = -1.0e6f;

    /* err 和 valid 必须成对更新: 中断随时可能打断这里,
     * 若写到一半(新 err + 旧 valid)被中断取走, 会多算一拍错误输入。
     * 关中断只有几条指令, 50Hz 调用下代价可忽略。 */
    {
        uint32 primask = interrupt_global_disable();
        ctrl_err       = err;
        ctrl_err_valid = valid;
        interrupt_global_enable(primask);
    }
}

//在线改巡航速度
void ctrl_set_cruise(float rpm)
{
    if(rpm < CTRL_CRUISE_MIN) rpm = CTRL_CRUISE_MIN;
    if(rpm > CTRL_CRUISE_MAX) rpm = CTRL_CRUISE_MAX;
    ctrl_cruise = rpm;
}

float ctrl_get_cruise(void)
{
    return ctrl_cruise;
}

// 速度提示系数: 视觉判环岛/岔路时压速度, 控制层不知道原因
//启动: 只置请求标志, 真正的状态切换在 TIM7 的 ctrl_tick 里做
void ctrl_start(void)
{
    uint32 primask = interrupt_global_disable();
    ctrl_req |= CTRL_REQ_START;
    interrupt_global_enable(primask);
}

//停车: 只置请求标志, 下一拍 TIM7 里执行(刹车 + 清 PID + 回 IDLE)
//不在这里直接调 chassis_stop: 会和在跑的 chassis_tick 抢电机/PID 状态
void ctrl_stop(void)
{
    uint32 primask = interrupt_global_disable();
    ctrl_req |= CTRL_REQ_STOP;
    interrupt_global_enable(primask);
}

/*
 * 分段增益: 按 |err| 在 KP_LO 和 KP_HI 之间线性插值
 *
 * 直道小增益防抖, 弯道大增益保转向能力. 详见 ctrl.h 的说明.
 * 用线性过渡而不是硬切换, 避免 err 跨阈值时增益突跳引起一帧抖动.
 */
static float ctrl_turn_kp(float err)
{
    float a = (err < 0.0f) ? -err : err;

    /* 低增益区: 直道和微小修正 */
    if(a <= CTRL_TURN_ERR_LO)
    {
        return tune_get(TUNE_KP_LO);
    }
    /* 高增益区: 大弯和直角弯 */
    if(a >= CTRL_TURN_ERR_HI)
    {
        return tune_get(TUNE_KP_HI);
    }
    /* 过渡区: 线性插值 */
    return tune_get(TUNE_KP_LO)
         + (tune_get(TUNE_KP_HI) - tune_get(TUNE_KP_LO))
           * (a - CTRL_TURN_ERR_LO)
           / (CTRL_TURN_ERR_HI - CTRL_TURN_ERR_LO);
}

/*
 * 软启动系数: 0 -> 1 线性上升
 * 起步时如果目标直接给 500 RPM, 速度环的增量式 PID 会一次性打满占空比,
 * 车会"蹭"一下窜出去. 300ms 斜坡让加速度可控, 也顺便避免积分饱和.
 */
static float ctrl_ramp_factor(void)
{
    if(ctrl_ramp_ms >= CTRL_RAMP_MS)
    {
        return 1.0f;
    }
    return (float)ctrl_ramp_ms / (float)CTRL_RAMP_MS;
}

void ctrl_tick(void)
{
    float turn      = 0.0f;
    float base;
    float turn_lim;
    float left;
    float right;

    //---- 0) 执行主循环送来的启动/停车请求 ----
    //放在状态机之前: 本拍请求启动, 本拍立刻开始算; 请求停车, 本拍直接
    //走 IDLE 分支返回。先 start 后 stop, 保证"同一拍内停车优先"。
    {
        uint8 req;

        /* 读清必须原子: 若先读后写中间被主循环置位, 新请求会被一起抹掉。
         * 关中断只有两条指令, 20ms 一次的代价可忽略。 */
        {
            uint32 primask = interrupt_global_disable();
            req = ctrl_req;
            ctrl_req = 0;
            interrupt_global_enable(primask);
        }

        if(req)
        {
            if(req & CTRL_REQ_START)
            {
                if(CTRL_STATE_IDLE == ctrl_state || CTRL_STATE_STOPPED == ctrl_state)
                {
                    //清 PID 历史, 否则上一次停车残留的积分会让起步窜一下
                    PID_Reset(&ctrl_turn_pid);
                    ctrl_lost_ms    = 0;
                    ctrl_ramp_ms    = 0;
                    ctrl_state      = CTRL_STATE_RUN;
                }
            }

            if(req & CTRL_REQ_STOP)
            {
                ctrl_state      = CTRL_STATE_IDLE;
                ctrl_lost_ms    = 0;
                ctrl_ramp_ms    = 0;
                ctrl_out_left   = 0.0f;
                ctrl_out_right  = 0.0f;
                ctrl_out_turn   = 0.0f;

                //底盘一起停. 这里在 TIM7 上下文里, 不会和 chassis_tick 重入.
                chassis_stop();
            }
        }
    }

    switch(ctrl_state)
    {
        case CTRL_STATE_IDLE:
        {
            //没启动: 什么都不做, 交给底盘和菜单页
            return;
        }

        case CTRL_STATE_STOPPED:
        {
            //已经刹停, 等用户手动重启
            return;
        }

        case CTRL_STATE_RUN:
        case CTRL_STATE_LOST:
        default:
        {
            break;
        }
    }

    /* ---- 0.5) 环岛状态机 ----
     *
     * 放在这里而不是 ctrl_set_error() 里:
     *   ctrl_set_error 由主循环每帧(约 20ms)调, 而环岛判据需要稳定节拍。
     *   这里由 TIM7 每 4ms 调一次, 计时器 s_running_ms 才准。
     *
     * 放在状态判断之后而不是之前:
     *   IDLE/STOPPED 已经 return 了, 车都没跑, 不需要推进环岛状态。
     *
     * 它只读 yaw、写内部状态, 不碰 moto 输出。
     * 出岛请求由 task_track 主循环取走并转告视觉。
     */
    island_tick();

    //---- 1) 速度规划 (丢线时也要, 否则会超速) ----
    base = ctrl_cruise;
    /* 环岛速度缩放：RUNNING 75%、AFTER 85%->100%。
     * 视觉的 g_speed_pct 从来没接到这里过，环岛降速一直是断的，
     * 这里把它补上。 */
    base *= island_get_speed_scale();


    //---- 2) 丢线判定与转向决策 ----
    if(!ctrl_err_valid)
    {
        ctrl_lost_ms += CTRL_PERIOD_MS;

        if(ctrl_lost_ms >= CTRL_LOST_STOP_MS)
        {
            //阶段 3: 丢线太久, 刹停.
            //输出也一并清零: 否则页面会显示进 STOP 前最后一拍的目标转速,
            //看起来像电机还在转, 容易误判成"没刹住".
            //异常情况(摄像头掉了/线彻底没了)必须停, 否则车会跑飞出赛道.
            ctrl_state = CTRL_STATE_STOPPED;

            ctrl_out_left  = 0.0f;
            ctrl_out_right = 0.0f;
            ctrl_out_turn  = 0.0f;

            chassis_stop();
            return;
        }

        if(ctrl_lost_ms <= CTRL_LOST_HOLD_MS)
        {
            //阶段 1 HOLD: 短暂丢线(反光/阴影), 线大概率还在原方向.
            //直接复用上一拍的 turn, 不调 PID_Calc.
            //为什么不调: 视觉丢线时 err 是旧值且每帧不变,
            //           反复喂同一个 err 会让 err_sum 持续累加(开 I 后就是 windup),
            //           而且 Kd*(e-e_last) 恒为 0, D 项失效.
            turn = ctrl_out_turn;
            base *= CTRL_LOST_HOLD_SLOW;
        }
        else
        {
            //阶段 2 RECOVER: 100ms 还没找到, 线真的不在了.
            //回正 + 降速, 让车先直行, 同时给视觉一个重新找线的机会.
            //此时 PID 状态已无意义, 重置防止恢复时输出跳变.
            /* 保持转向惯性：前一次的转向量按 15% 衰减，
             * 而不是直接归零。拐弯中丢时扔线，有转向惯性才能找回来。 */
            turn = ctrl_out_turn * 0.85f;
            base *= CTRL_LOST_SLOW_RATIO;

        }

        ctrl_state = CTRL_STATE_LOST;
    }
    else
    {
        //---- 正常循迹 ----
        if(CTRL_STATE_LOST == ctrl_state)
        {
            //刚从丢线恢复: PID 状态是旧的, 重置一次避免输出跳变
            PID_Reset(&ctrl_turn_pid);
        }
        ctrl_lost_ms = 0;
        ctrl_state   = CTRL_STATE_RUN;

        //起步斜坡: 从死区速度爬到目标速度, 用 CTRL_RAMP_MS 平滑过渡.
        //不从 0 开始: 低于死区的那段电机根本不转, 白占斜坡时间,
        //            真正能动的时候斜坡已经用完, 等于没做平滑.
        {
            float ramp = ctrl_ramp_factor();

            base = CTRL_WHEEL_MIN_RPM + (base - CTRL_WHEEL_MIN_RPM) * ramp;
        }
        if(ctrl_ramp_ms < CTRL_RAMP_MS)
        {
            ctrl_ramp_ms += CTRL_PERIOD_MS;
        }

        /* 分段增益: 每帧按当前 err 重新设 KP.
         * 直道 err 小 -> KP=2.5 不放大噪声; 弯道 err 大 -> KP 最高 7.0. */
        PID_SetTuning(&ctrl_turn_pid, ctrl_turn_kp(ctrl_err),
                      0.0f, 0.0f);

        //标准写法: target=0, feedback=err. 内部 e = -err.
        //err > 0 -> e < 0 -> turn < 0 -> left 慢 -> 右转
        ctrl_turn_pid.out_max = tune_get(TUNE_TURN_MAX);
        turn = PID_Calc(&ctrl_turn_pid, 0.0f, ctrl_err);
    }

    /* PID 输出兜底: KP/KI/KD 万一被 VOFA 设成异常值, 或者 err 里
     * 混进了 NaN, turn 可能就是脏的. 这里再挡一次, 保证下发给底盘的
     * 一定是有限数. */
    /* ---- 3.5) 环岛出岛转向覆盖（学学长 CMD_AFTER）----
     *
     * 出岛后 250ms 内，视觉看到的是环岛出口的弧线，err 会指向弧线方向。
     * 这段时间改用 yaw 闭环把车头拉回"入环前的朝向"，直行穿出去。
     *
     * 混合权重 w：
     *   0   = 纯视觉（正常巡线 / 环内）
     *   1   = 纯 yaw 锁定（出岛中）
     * yaw_turn 自己会在最后 100ms 线性衰减，所以交接是连续的。
     */
    {
        float yaw_turn = 0.0f;
        float w = island_get_turn_blend(&yaw_turn);

        if (w > 0.0f)
        {
            if (w >= 1.0f)
            {
                turn = yaw_turn;            /* 纯 yaw 锁定 */
            }
            else
            {
                turn = turn * (1.0f - w) + yaw_turn * w;
            }
        }
    }

    if(!(turn == turn))
    {
        turn = 0.0f;
    }
    if(turn >  1.0e6f) turn =  1.0e6f;
    if(turn < -1.0e6f) turn = -1.0e6f;

    // Limit steering differential around the fixed cruise target.
    turn_lim = ctrl_cruise - CTRL_INNER_WHEEL_MIN_RPM;
    if(turn_lim < 0.0f)
    {
        turn_lim = 0.0f;
    }
    if(turn_lim > tune_get(TUNE_TURN_MAX))
    {
        turn_lim = tune_get(TUNE_TURN_MAX);
    }
    if(turn >  turn_lim) turn =  turn_lim;
    if(turn < -turn_lim) turn = -turn_lim;

    //---- 4) 差速合成 ----
    left  = base - turn;
    right = base + turn;

    //双保险: 理论上压缩后不会低于 WHEEL_MIN, 但丢线/斜坡等分支
    //可能绕过上面的计算, 这里再挡一次
    if(left  < 0.0f) left  = 0.0f;
    if(right < 0.0f) right = 0.0f;

    ctrl_out_left  = left;
    ctrl_out_right = right;
    ctrl_out_turn  = turn;

    //---- 5) 下发 ----
    chassis_set_target((int32)left, (int32)right);
}

ctrl_state_enum ctrl_get_state(void)
{
    return ctrl_state;
}

void ctrl_get_output(float *left_rpm, float *right_rpm, float *turn)
{
    if(left_rpm)  *left_rpm  = ctrl_out_left;
    if(right_rpm) *right_rpm = ctrl_out_right;
    if(turn)      *turn      = ctrl_out_turn;
}

float ctrl_get_error(void)
{
    return ctrl_err;
}

float ctrl_get_turn(void)
{
    return ctrl_out_turn;
}
