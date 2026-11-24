#include "task_motor.h"
#include "app_ui.h"
#include "algorithm/chassis.h"
#include "task_vofa.h"

//七行的序号
#define ROW_L_TGT       (0)
#define ROW_L_ACT       (1)
#define ROW_R_TGT       (2)
#define ROW_R_ACT       (3)
#define ROW_L_CNT       (4)
#define ROW_R_CNT       (5)

//两种模式
#define MODE_CLOSED     (0)         //闭环: 按键调 RPM
#define MODE_OPEN       (1)         //开环: 按键调占空比

static uint8  motor_sel   = ENC_LEFT;   //当前选中的一侧
static uint8  motor_mode  = MODE_CLOSED;//当前模式

static int32  motor_target[2] = {0, 0}; //闭环目标转速(RPM)
static int32  motor_duty[2]   = {0, 0}; //开环占空比(0~10000)

static uint32 motor_last_tick  = 0;     //上次刷屏时刻
static uint32 motor_mid_press  = 0;     //中键按下的时刻, 0=没按

//带符号的整数转成 "123 rpm" / "1234" 形式
//ui_num_to_str 只处理无符号数, 负数在这里自己补 '-'
static void motor_fmt(int32 value, char *buf, uint8 with_rpm)
{
    uint8 n = 0;
    uint32 v;

    if(value < 0)
    {
        buf[n ++] = '-';
        v = (uint32)(-value);
    }
    else
    {
        v = (uint32)value;
    }

    n += ui_num_to_str(v, &buf[n]);

    if(with_rpm)
    {
        buf[n ++] = ' ';
        buf[n ++] = 'r';
        buf[n ++] = 'p';
        buf[n ++] = 'm';
    }

    buf[n] = '\0';
}

//画设定值那一行(闭环显示 TGT/rpm, 开环显示 DUTY)
static void motor_draw_set(uint8 row)
{
    char  buf[16];
    uint8 is_left = (row == ROW_L_TGT) ? 1 : 0;
    uint8 side    = is_left ? ENC_LEFT : ENC_RIGHT;
    uint8 sel     = (motor_sel == side) ? 1 : 0;

    if(MODE_OPEN == motor_mode)
    {
        motor_fmt(motor_duty[side], buf, 0);
        ui_draw_row_text(MOTOR_ROW_Y0 + row * MOTOR_ROW_STEP,
                         sel ? (is_left ? ">L DUTY" : ">R DUTY")
                             : (is_left ? " L DUTY" : " R DUTY"),
                         buf, 0);
    }
    else
    {
        motor_fmt(motor_target[side], buf, 1);
        ui_draw_row_text(MOTOR_ROW_Y0 + row * MOTOR_ROW_STEP,
                         sel ? (is_left ? ">L TGT" : ">R TGT")
                             : (is_left ? " L TGT" : " R TGT"),
                         buf, 0);
    }
}

//画实际转速那一行
//开环模式下也照常显示, 用来验证编码器是不是在工作
static void motor_draw_act(uint8 row, uint8 side)
{
    char buf[16];

    motor_fmt(chassis_get_speed(side), buf, 1);
    ui_draw_row_text(MOTOR_ROW_Y0 + row * MOTOR_ROW_STEP,
                     (side == ENC_LEFT) ? " L ACT" : " R ACT", buf, 0);
}

//画累计计数那一行
//
//计数必须按有符号显示:
//  hw_encoder_get_total 返回 int32, 编码器方向标反时会是负数.
//  原来强转成 uint32, -689389 会显示成 4294277907,
//  看起来像"计数暴涨", 实际上只是反向, 很容易误导排查.
static void motor_draw_cnt(uint8 row, uint8 side)
{
    char buf[16];

    motor_fmt(hw_encoder_get_total(side), buf, 0);
    ui_draw_row_text(MOTOR_ROW_Y0 + row * MOTOR_ROW_STEP,
                     (side == ENC_LEFT) ? " L CNT" : " R CNT", buf, 0);
}

//刷全部七行数值
static void motor_draw_values(void)
{
    motor_draw_set(ROW_L_TGT);
    motor_draw_act(ROW_L_ACT, ENC_LEFT);
    motor_draw_set(ROW_R_TGT);
    motor_draw_act(ROW_R_ACT, ENC_RIGHT);
    motor_draw_cnt(ROW_L_CNT, ENC_LEFT);
    motor_draw_cnt(ROW_R_CNT, ENC_RIGHT);
}

//按当前运行状态画标题
static void motor_draw_title(void)
{
    if(MODE_OPEN == motor_mode)
    {
        ui_draw_title((CHASSIS_STATE_OPEN == chassis_get_state()) ? "OPEN   RUN" : "MOTOR OPEN");
    }
    else
    {
        ui_draw_title((CHASSIS_STATE_RUN == chassis_get_state()) ? "MOTOR  RUN" : "MOTOR TEST");
    }
}

//把当前模式的设定值下发给底盘
static void motor_apply(void)
{
    if(MODE_OPEN == motor_mode)
    {
        chassis_open_loop((int16)motor_duty[ENC_LEFT], (int16)motor_duty[ENC_RIGHT]);
    }
    else
    {
        chassis_set_target(motor_target[ENC_LEFT], motor_target[ENC_RIGHT]);
    }
}

//当前是否处于运行状态
static uint8 motor_is_running(void)
{
    chassis_state_enum st = chassis_get_state();

    if(MODE_OPEN == motor_mode)
    {
        return (CHASSIS_STATE_OPEN == st) ? 1 : 0;
    }
    return (CHASSIS_STATE_RUN == st) ? 1 : 0;
}

//切换闭环/开环
static void motor_switch_mode(void)
{
    chassis_stop();                     //切模式前先停车, 避免占空比突然跳变

    motor_mode = (MODE_CLOSED == motor_mode) ? MODE_OPEN : MODE_CLOSED;

    motor_draw_title();
    motor_draw_values();                //两行设定值的标题会跟着变
    ui_draw_footer_line(1, (MODE_OPEN == motor_mode) ? "M:Run HoldM:Closed"
                                                     : "M:Run HoldM:Mode");
}

//调整选中那一路的设定值
static void motor_adjust(int32 step)
{
    int32 next;
    uint8 side = motor_sel;

    if(MODE_OPEN == motor_mode)
    {
        next = motor_duty[side] + step;

        if(next > MOTOR_DUTY_MAX)
        {
            next = MOTOR_DUTY_MAX;
        }
        if(next < -MOTOR_DUTY_MAX)
        {
            next = -MOTOR_DUTY_MAX;
        }
        motor_duty[side] = next;
    }
    else
    {
        next = motor_target[side] + step;

        if(next > CHASSIS_TARGET_MAX)
        {
            next = CHASSIS_TARGET_MAX;
        }
        if(next < -CHASSIS_TARGET_MAX)
        {
            next = -CHASSIS_TARGET_MAX;
        }
        motor_target[side] = next;
    }

    //已经处于运行状态时, 改完立刻生效
    if(motor_is_running())
    {
        motor_apply();
    }

    motor_draw_set((ENC_LEFT == side) ? ROW_L_TGT : ROW_R_TGT);
}

//进页面
void task_motor_show(void)
{
    motor_sel       = ENC_LEFT;
    motor_mode      = MODE_CLOSED;
    motor_target[0] = 0;
    motor_target[1] = 0;
    motor_duty[0]   = 0;
    motor_duty[1]   = 0;
    motor_last_tick = ui_tick_get();
    motor_mid_press = 0;

    vofa_release();                     //交还底盘, 防止 VOFA 同时写目标
    hw_encoder_reset();                 //计数清零, 方便观察编码器是否工作
    chassis_stop();                     //一进来必须是停车状态

    ui_clear();
    motor_draw_title();
    motor_draw_values();

    ui_draw_footer_line(0, "S1/S2:Sel U/D:+ -");
    ui_draw_footer_line(1, "M:Run HoldM:Mode");
}

//按键处理(中键的长短按在 run 里处理)
void task_motor_key(key_id_enum id)
{
    switch(id)
    {
        case KEY_ID_S1:                         //选左轮
        {
            motor_sel = ENC_LEFT;
            motor_draw_set(ROW_L_TGT);
            motor_draw_set(ROW_R_TGT);          //另一行的 '>' 要擦掉
            break;
        }

        case KEY_ID_S2:                         //选右轮
        {
            motor_sel = ENC_RIGHT;
            motor_draw_set(ROW_L_TGT);
            motor_draw_set(ROW_R_TGT);
            break;
        }

        case KEY_ID_UP:                         //加
        {
            motor_adjust((MODE_OPEN == motor_mode) ? MOTOR_DUTY_STEP : MOTOR_TARGET_STEP);
            break;
        }

        case KEY_ID_DOWN:                       //减
        {
            motor_adjust(-((MODE_OPEN == motor_mode) ? MOTOR_DUTY_STEP : MOTOR_TARGET_STEP));
            break;
        }

        default: break;
    }
}

//运行: 处理中键长短按 + 定时刷数值
void task_motor_run(void)
{
    uint8 id = (uint8)hw_key_get();

    //中键长短按: 按住 MOTOR_HOLD_MODE_MS 算长按(切模式), 松开算短按(启停)
    if(KEY_ID_MID == (key_id_enum)id)
    {
        if(0 == motor_mid_press)                    //刚按下
        {
            motor_mid_press = ui_tick_get();
            if(0 == motor_mid_press)                //恰好为 0 会被当成没按
            {
                motor_mid_press = 1;
            }
        }
        else if(ui_tick_get() - motor_mid_press >= MOTOR_HOLD_MODE_MS)
        {
            motor_mid_press = 0;                    //清掉, 松手时不会再触发短按
            motor_switch_mode();
        }

        return;                                     //按住期间不刷屏, 免得拖慢主循环
    }

    //松手时还记着按下时刻 -> 没熬到长按, 算短按
    if(0 != motor_mid_press)
    {
        motor_mid_press = 0;

        if(motor_is_running())
        {
            chassis_stop();
        }
        else
        {
            motor_apply();
        }
        motor_draw_title();
    }

    if(ui_tick_get() - motor_last_tick >= MOTOR_REFRESH_MS)
    {
        motor_last_tick = ui_tick_get();
        motor_draw_values();
    }
}

//离开页面: 必须停车, 否则轮子会一直转着出页面
void task_motor_leave(void)
{
    chassis_stop();

    motor_target[0] = 0;
    motor_target[1] = 0;
    motor_duty[0]   = 0;
    motor_duty[1]   = 0;
    motor_mid_press = 0;
}
