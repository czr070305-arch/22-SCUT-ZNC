#include "chassis.h"
#include "pid.h"
#include "hw_flash.h"

//左右各一个速度环
//用增量式: 速度环是一阶对象, 增量式启动平滑不跳变, 也不会因为积分饱和而冲过头
static PID_t chassis_pid[2];

//可调参数
static t_chassis_param chassis_param;

//目标转速(RPM)
static int32 chassis_target[2] = {0, 0};

//实际转速(RPM), 每个周期从编码器更新
static int32 chassis_speed[2] = {0, 0};

//当前 PID 输出(占空比), 留给上层调试观察
static int16 chassis_out[2] = {0, 0};

//滤波后的速度, 单位 0.01RPM
//用 x100 的整数存, 避免在中断里做浮点运算
//初始化成 0 表示"还没采过", 第一次采样直接赋值不滤波
static int32 chassis_speed_x100[2] = {0, 0};
static uint8 chassis_speed_init[2] = {0, 0};

//控制周期计数, 每进一次 chassis_tick 加 1
//上位机等需要"和速度环对齐"的定时都用它, 别用主循环软件计时:
//主循环会被 TFT 刷新拖慢, 软件计时会漂
static volatile uint32 chassis_tick_cnt = 0;

//开环模式下的占空比
static int16 chassis_open_duty[2] = {0, 0};

//运行状态
static chassis_state_enum chassis_state = CHASSIS_STATE_STOP;

//把 0.01RPM 的定点数四舍五入成整数 RPM
//四舍五入而不是直接截断, 否则低速时读数会系统性偏小
static int32 rpm_x100_to_int(int32 rpm_x100)
{
    if(rpm_x100 >= 0)
    {
        return (rpm_x100 + 50) / 100;
    }
    return -((-rpm_x100 + 50) / 100);
}

//初始化
void chassis_init(void)
{
    uint8 i;

    hw_motor_init();                    //电机先初始化, EN 默认 0 = 刹车
    hw_encoder_init();                  //编码器

    //参数从 Flash 读(由 flash_param_load 提前装进 g_param)
    //读不到时 g_param 已经是 0, 也就是没有 P 没有 I, 电机不会动
    //左右轮各用各的参数(两个电机的机械/编码器特性不完全一致)
    chassis_param.l_kp      = g_param.l_kp;
    chassis_param.l_ki      = g_param.l_ki;
    chassis_param.l_kd      = g_param.l_kd;
    chassis_param.r_kp      = g_param.r_kp;
    chassis_param.r_ki      = g_param.r_ki;
    chassis_param.r_kd      = g_param.r_kd;
    chassis_param.out_max   = CHASSIS_OUT_MAX;
    chassis_param.dead_zone = CHASSIS_DEAD_ZONE;

    for(i = 0; i < 2; i ++)
    {
        float kp = (ENC_LEFT == i) ? chassis_param.l_kp : chassis_param.r_kp;
        float ki = (ENC_LEFT == i) ? chassis_param.l_ki : chassis_param.r_ki;
        float kd = (ENC_LEFT == i) ? chassis_param.l_kd : chassis_param.r_kd;

        //增量式 PID, 输出限幅到满占空比
        //int_max 对增量式无效(位置式才靠它抗积分饱和), 给 0
        PID_Init(&chassis_pid[i],
                 kp, ki, kd,
                 chassis_param.out_max,
                 0.0f,
                 chassis_param.dead_zone,
                 PID_MODE_INC);

        chassis_target[i]    = 0;
        chassis_speed[i]     = 0;
        chassis_out[i]       = 0;
        chassis_open_duty[i] = 0;
    }

    chassis_state = CHASSIS_STATE_STOP;

    //启动闭环节拍, 之后 chassis_tick 会被定时器中断反复调用
    pit_ms_init(CHASSIS_PIT_TIM, CHASSIS_PERIOD_MS);
}

//设定目标转速
void chassis_set_target(int32 left_rpm, int32 right_rpm)
{
    if(left_rpm >  CHASSIS_TARGET_MAX) left_rpm  =  CHASSIS_TARGET_MAX;
    if(left_rpm < -CHASSIS_TARGET_MAX) left_rpm  = -CHASSIS_TARGET_MAX;
    if(right_rpm >  CHASSIS_TARGET_MAX) right_rpm =  CHASSIS_TARGET_MAX;
    if(right_rpm < -CHASSIS_TARGET_MAX) right_rpm = -CHASSIS_TARGET_MAX;

    chassis_target[ENC_LEFT]  = left_rpm;
    chassis_target[ENC_RIGHT] = right_rpm;

    //从停车状态切到闭环时, 先把 PID 历史清干净
    //否则上一次停车残留的累加值会让电机一启动就窜一下
    if(CHASSIS_STATE_RUN != chassis_state)
    {
        PID_Reset(&chassis_pid[ENC_LEFT]);
        PID_Reset(&chassis_pid[ENC_RIGHT]);
    }

    chassis_state = CHASSIS_STATE_RUN;
}

//读目标转速
int32 chassis_get_target(uint8 side)
{
    return chassis_target[side & 0x01];
}

//读实际转速
int32 chassis_get_speed(uint8 side)
{
    return chassis_speed[side & 0x01];
}

//开环模式
void chassis_open_loop(int16 left_duty, int16 right_duty)
{
    chassis_open_duty[ENC_LEFT]  = left_duty;
    chassis_open_duty[ENC_RIGHT] = right_duty;
    chassis_state                = CHASSIS_STATE_OPEN;
}

//停车
void chassis_stop(void)
{
    uint8 i;

    for(i = 0; i < 2; i ++)
    {
        chassis_target[i]    = 0;
        chassis_out[i]       = 0;
        chassis_open_duty[i] = 0;
        PID_Reset(&chassis_pid[i]);
    }

    hw_motor_stop();
    chassis_state = CHASSIS_STATE_STOP;

    //清掉滤波历史, 下次启动从当前实际转速重新开始
    //不清的话上次停车前的残留值会让第一次采样跳一下
    for(i = 0; i < 2; i ++)
    {
        chassis_speed_x100[i] = 0;
        chassis_speed_init[i] = 0;
    }
}

//当前状态
chassis_state_enum chassis_get_state(void)
{
    return chassis_state;
}

//在线调参
void chassis_set_param(uint8 side, float kp, float ki, float kd)
{
    //PID 对象本身
    chassis_pid[side & 0x01].Kp = kp;
    chassis_pid[side & 0x01].Ki = ki;
    chassis_pid[side & 0x01].Kd = kd;

    //同步回参数表和全局表, 这样 SAVE 才能把这一侧的改动写进 Flash
    if(ENC_LEFT == (side & 0x01))
    {
        chassis_param.l_kp = kp;
        chassis_param.l_ki = ki;
        chassis_param.l_kd = kd;

        g_param.l_kp = kp;
        g_param.l_ki = ki;
        g_param.l_kd = kd;
    }
    else
    {
        chassis_param.r_kp = kp;
        chassis_param.r_ki = ki;
        chassis_param.r_kd = kd;

        g_param.r_kp = kp;
        g_param.r_ki = ki;
        g_param.r_kd = kd;
    }
}

//取 PID 输出
void chassis_get_param(uint8 side, float *kp, float *ki, float *kd)
{
    if(ENC_LEFT == (side & 0x01))
    {
        if(0 != kp) *kp = chassis_param.l_kp;
        if(0 != ki) *ki = chassis_param.l_ki;
        if(0 != kd) *kd = chassis_param.l_kd;
    }
    else
    {
        if(0 != kp) *kp = chassis_param.r_kp;
        if(0 != ki) *ki = chassis_param.r_ki;
        if(0 != kd) *kd = chassis_param.r_kd;
    }
}

//控制周期计数
uint32 chassis_get_tick(void)
{
    return chassis_tick_cnt;
}

int16 chassis_get_output(uint8 side)
{
    return chassis_out[side & 0x01];
}

//节拍处理, TIM7 中断里每 CHASSIS_PERIOD_MS(默认10ms) 调一次
void chassis_tick(void)
{
    uint8 i;
    float out;

    chassis_tick_cnt ++;

    //1) 采样编码器, 更新实际转速
    hw_encoder_update();

    for(i = 0; i < 2; i ++)
    {
        int32 raw = hw_encoder_get_rpm_x100((uint8)i);

#if (CHASSIS_FILTER_SHIFT > 0)
        if(0 == chassis_speed_init[i])
        {
            //第一次采样直接赋值, 否则要从 0 慢慢爬上来, 启动会有个假滞后
            chassis_speed_x100[i] = raw;
            chassis_speed_init[i] = 1;
        }
        else
        {
            //一阶低通: 新值占 1/(2^shift)
            //差值为负时右移会向负无穷取整, 所以用减法形式算
            chassis_speed_x100[i] += (raw - chassis_speed_x100[i]) >> CHASSIS_FILTER_SHIFT;
        }

        chassis_speed[i] = rpm_x100_to_int(chassis_speed_x100[i]);
#else
        chassis_speed[i] = rpm_x100_to_int(raw);
        chassis_speed_x100[i] = raw;
        chassis_speed_init[i] = 1;
#endif
    }

    //2) 按当前状态输出
    switch(chassis_state)
    {
        case CHASSIS_STATE_OPEN:
        {
            //开环: 不经过 PID, 直接给占空比
            hw_motor_set(chassis_open_duty[ENC_LEFT], chassis_open_duty[ENC_RIGHT]);

            for(i = 0; i < 2; i ++)
            {
                chassis_out[i] = chassis_open_duty[i];
            }
            break;
        }

        case CHASSIS_STATE_RUN:
        {
            for(i = 0; i < 2; i ++)
            {
                out = PID_Calc(&chassis_pid[i],
                               (float)chassis_target[i],
                               (float)chassis_speed[i]);

                chassis_out[i] = (int16)out;
            }

            hw_motor_set(chassis_out[ENC_LEFT], chassis_out[ENC_RIGHT]);
            break;
        }

        case CHASSIS_STATE_STOP:
        default:
        {
            //停车状态: 目标清零并刹车
            //PID 的清理已经在 chassis_stop() 里做了, 这里不重复
            //也不能每 10ms 都调 PID_Reset, 那是无谓的开销
            for(i = 0; i < 2; i ++)
            {
                chassis_out[i] = 0;
            }

            hw_motor_set(0, 0);
            break;
        }
    }
}
