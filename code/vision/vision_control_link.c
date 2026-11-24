/* ==========================================================================
 *  vision_control_link.c —— 视觉结果 → 控制目标量（"视觉与控制联动"的核心）
 *  --------------------------------------------------------------------------
 *  【它解决什么问题】
 *  视觉模块只输出一个东西：转向误差 err（单位是"像素当量"，右为正）。
 *  这个小车要跑起来，还得把它变成"左右轮的占空比/舵机角度"，
 *  并且处理三件事：丢线怎么办、环岛要不要减速、误差要不要微分阻尼。
 *
 *  【重要：本文件默认"只算目标量，不直接驱动电机"】
 *  你现在 MRS 工程里的控制部分（assessment_app.c）没有电机输出，
 *  按你的要求我没有去改它。所以这里做成两层：
 *
 *      第一层（默认开启）：算出 steer / speed 目标，存进全局变量，
 *                          控制层随时可以读；
 *      第二层（默认关闭）：VISION_LINK_DRIVE_MOTOR = 1 时直接写 PWM，
 *                          给你"还没有控制代码、想先把车跑起来"时用。
 *
 *  两种接法（任选其一，详见文末说明）：
 *      A. 你的控制层自己读 vision_link_steer()/vision_link_speed()
 *         （推荐：把 err 当成你 PID 的输入，速度环/方向环都你自己做）
 *      B. 把 VISION_LINK_DRIVE_MOTOR 改成 1，用本文件的差速开环输出
 *         （只适合先把车"跑起来看方向对不对"，正式跑要加速度闭环）
 * ========================================================================== */
#include "vision.h"

/* --------------------------------------------------------------------------
 * 联动参数（全部集中在这里，控制层调参只改这一段）
 * -------------------------------------------------------------------------- */
#define LINK_KP             (5.2f)      /* 比例：每个像素误差给多少占空比
                                         * err 限幅 ±110，所以 KP=5.2 时
                                         * 满偏 ≈ 570/1000，够用且不爆 */
#define LINK_KD             (2.4f)      /* 微分阻尼：抑制"打过头来回摆" */
#define LINK_ERR_DEADBAND   (2.0f)      /* 误差死区（像素）：直道上别抖 */
#define LINK_STEER_MAX      (620)       /* 转向分量上限（占空比 0~1000） */
#define LINK_SPEED_BASE     (560)       /* 直道基础速度（0~1000） */
#define LINK_SPEED_CURVE    (3.2f)      /* 每 1px 误差降多少速度 */
#define LINK_SPEED_MIN      (260)       /* 弯道最低速度，别停死 */
#define LINK_SPEED_ISLAND   (340)       /* 环岛速度上限（进岛必须慢） */
#define LINK_SPEED_FORK     (420)       /* 岔路速度上限 */
#define LINK_LOST_HOLD      (6)         /* 丢线几帧内沿用上次转向 */
#define LINK_LOST_STOP      (25)        /* 连续丢线超过这么多帧 → 停车 */

/* --------------------------------------------------------------------------
 * 输出层开关
 *  0 = 只算目标量（安全，不会动电机）
 *  1 = 直接驱动双路 8701（需要按你的实际接线填 PWM 通道）
 * -------------------------------------------------------------------------- */
#define VISION_LINK_DRIVE_MOTOR     (0)

/* 若打开上面的开关，请按原理图和你的驱动板接线，改下面 4 个宏。
 *  原理图第 3 页"电机驱动隔离"网络名：
 *      LPWM1_E9 / RPWM1_E8 / LPWM2_E11 / RPWM2_E10
 *  这两组是"左轮 IN1/IN2、右轮 IN1/IN2"，具体哪个是 E9/E8 请以实物飞线为准。
 *  下面用占位的 PWM 通道名，编译前必须改成 zf_driver_pwm.h 里真实存在的项。
 * -------------------------------------------------------------------------- */
#if (VISION_LINK_DRIVE_MOTOR)
    /* 示例：把 TIM1 的两路 PWM 当"左轮 IN1 / 右轮 IN1"，
     *       方向由另外两根 GPIO 控制（8701 的 IN2 拉低即可单向转）。 */
    #define LINK_PWM_LEFT       (TIM1_PWM_MAP0_CH1_A8)     /* ← 改成你的实际通道 */
    #define LINK_PWM_RIGHT      (TIM1_PWM_MAP0_CH2_A9)     /* ← 改成你的实际通道 */
    #define LINK_PWM_FREQ       (20000)                    /* 20kHz，避开电机啸叫 */
#endif

/* --------------------------------------------------------------------------
 * 对外的目标量（控制层直接读）
 * -------------------------------------------------------------------------- */
static int   s_link_steer = 0;          /* -1000~+1000，右为正 */
static int   s_link_speed = 0;          /* 0~1000 */
static int   s_link_left  = 0;          /* 左轮占空比 0~1000 */
static int   s_link_right = 0;          /* 右轮占空比 0~1000 */
static float s_last_err = 0.0f;
static int   s_have_last = 0;
#if (VISION_LINK_DRIVE_MOTOR)
static int   s_pwm_ready = 0;           /* PWM 只在真正驱动电机时才需要初始化 */
#endif

/* --------------------------------------------------------------------------
 * 输出钩子（弱函数）
 *
 *  默认实现什么都不做，这样"只算目标量"时绝对不会动电机。
 *  你想自己接管输出（比如走自己的 PID / 舵机），
 *  就在任意一个 .c 文件里定义一个同名函数把它覆盖掉，不必改本文件：
 *
 *      void vision_link_output(int left, int right)
 *      {
 *          // left/right 是 0~1000 的占空比，你自己决定怎么写
 *      }
 * -------------------------------------------------------------------------- */
__attribute__((weak)) void vision_link_output(int left, int right)
{
#if (VISION_LINK_DRIVE_MOTOR)
    extern void vision_link_apply_pwm(int left, int right);
    vision_link_apply_pwm(left, right);
#else
    (void)left;
    (void)right;
#endif
}

/* --------------------------------------------------------------------------
 * 每帧调用：vision_task() 处理完一帧后自动调
 * -------------------------------------------------------------------------- */
void vision_link_update(void)
{
    const vision_out_t *o = vision_get_output();
    float err;
    float d;
    float steer_f;
    int   speed;

    /* ---- 1) 丢线处理 ---- */
    if (!o->err_valid) {
        if (o->lost_cnt <= LINK_LOST_HOLD) {
            /* 短暂丢线（十字横臂、反光）：目标量保持不变，再输出一次上次的值 */
            vision_link_output(s_link_left, s_link_right);
            return;
        }
        if (o->lost_cnt > LINK_LOST_STOP) {
            /* 彻底丢线：停车，防止冲出赛道 */
            s_link_left = 0;
            s_link_right = 0;
            s_link_steer = 0;
            s_link_speed = 0;
            vision_link_output(0, 0);
            return;
        }
        /* 半丢线：保持转向，速度降到最低档 */
        s_link_speed = LINK_SPEED_MIN;
        s_link_left = s_link_speed + s_link_steer / 2;
        s_link_right = s_link_speed - s_link_steer / 2;
        vision_link_output(s_link_left, s_link_right);
        return;
    }

    /* ---- 2) 误差与微分（带死区，防止直道抖） ---- */
    err = o->err;
    if (err > -LINK_ERR_DEADBAND && err < LINK_ERR_DEADBAND) {
        err = 0.0f;
    }
    if (!s_have_last) {
        s_last_err = err;
        s_have_last = 1;
    }
    d = err - s_last_err;
    s_last_err = err;

    /* ---- 3) PD → 转向分量（占空比单位） ---- */
    steer_f = LINK_KP * err + LINK_KD * d;
    if (steer_f > (float)LINK_STEER_MAX) {
        steer_f = (float)LINK_STEER_MAX;
    }
    if (steer_f < (float)-LINK_STEER_MAX) {
        steer_f = (float)-LINK_STEER_MAX;
    }
    s_link_steer = (int)steer_f;

    /* ---- 4) 速度：误差越大越慢；环岛/岔路再压一档 ---- */
    speed = LINK_SPEED_BASE - (int)(LINK_SPEED_CURVE * ((err < 0.0f) ? -err : err));
    if (speed < LINK_SPEED_MIN) {
        speed = LINK_SPEED_MIN;
    }
    if (o->island_on && (speed > LINK_SPEED_ISLAND)) {
        speed = LINK_SPEED_ISLAND;
    }
    if ((o->elem == 2) && (speed > LINK_SPEED_FORK)) {
        speed = LINK_SPEED_FORK;
    }
    s_link_speed = speed;

    /* ---- 5) 差速：右转 = 左轮快 / 右轮慢 ---- */
    s_link_left  = speed + s_link_steer;
    s_link_right = speed - s_link_steer;
    if (s_link_left < 0)    { s_link_left = 0; }
    if (s_link_left > 1000) { s_link_left = 1000; }
    if (s_link_right < 0)   { s_link_right = 0; }
    if (s_link_right > 1000){ s_link_right = 1000; }

    /* ---- 6) 交给输出层 ---- */
    vision_link_output(s_link_left, s_link_right);
}

/* --------------------------------------------------------------------------
 * 取目标量（控制层用这几个接口即可，不需要碰任何图像变量）
 * -------------------------------------------------------------------------- */
int vision_link_steer(void)
{
    return s_link_steer;
}

int vision_link_speed(void)
{
    return s_link_speed;
}

/* --------------------------------------------------------------------------
 * 只想先把车跑起来时用：开环 PWM 输出（VISION_LINK_DRIVE_MOTOR = 1 才生效）
 * -------------------------------------------------------------------------- */
#if (VISION_LINK_DRIVE_MOTOR)
void vision_link_apply_pwm(int left, int right)
{
    if (!s_pwm_ready) {
        pwm_init(LINK_PWM_LEFT, LINK_PWM_FREQ, 0);
        pwm_init(LINK_PWM_RIGHT, LINK_PWM_FREQ, 0);
        s_pwm_ready = 1;
    }
    /* 0~1000 → 0~PWM_DUTY_MAX(10000) */
    pwm_set_duty(LINK_PWM_LEFT,  (uint32)left  * PWM_DUTY_MAX / 1000);
    pwm_set_duty(LINK_PWM_RIGHT, (uint32)right * PWM_DUTY_MAX / 1000);
}
#endif

/* ==========================================================================
 *  怎么把它接进控制代码（三种情况，按你的实际情况选）
 *  --------------------------------------------------------------------------
 *  情况 1：你的控制层已经有"方向 PID / 舵机"，只是缺输入
 *      把 vision_link_steer() 当成 PID 的输入（或直接把 g_steer_err 乘系数）：
 *
 *          int steer = vision_link_steer();       // -1000~+1000
 *          servo_duty = SERVO_MID + steer * SERVO_SCALE;
 *
 *      速度环照旧用编码器做，视觉只提供"弯道该多慢"：
 *
 *          int v_target = vision_link_speed();    // 0~1000
 *
 *  情况 2：纯差速小车、没有舵机
 *      直接把 left/right 拿去驱动两路电机（要么用弱函数覆盖，
 *      要么把 VISION_LINK_DRIVE_MOTOR 改成 1 并填好 PWM 通道）。
 *
 *  情况 3：想自己写一套更复杂的策略
 *      只读 vision_get_output()：
 *          out->err        转向误差（像素）
 *          out->elem       0 正常 / 1 十字 / 2 岔路 / 3 环岛
 *          out->island_on  是否环岛中
 *          out->lap        第几圈（两圈走不同岔路时用）
 *          out->err_valid  0 表示本帧不可信
 * ========================================================================== */
