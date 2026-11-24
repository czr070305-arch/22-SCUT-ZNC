#include "hw_motor.h"

//把 -10000 ~ +10000 的单路指令拆成 PH 电平和 EN 占空比
//invert 用来处理电机线接反导致的方向相反
static void motor_output(gpio_pin_enum ph_pin, pwm_channel_enum en_pin,
                         int16 duty, uint8 invert)
{
    uint8 dir;
    uint16 pwm;

    //先限幅, 防止上层算出来的值越界把占空比写坏
    if(duty > MOTOR_DUTY_MAX)
    {
        duty = MOTOR_DUTY_MAX;
    }
    else if(duty < -MOTOR_DUTY_MAX)
    {
        duty = -MOTOR_DUTY_MAX;
    }

    //正数=前进, PH 给高; 负数=后退, PH 给低
    dir = (duty >= 0) ? 1 : 0;

    //硬件接反时在这里翻一次, 上层永远只表达"前进/后退"
    if(invert)
    {
        dir = !dir;
    }

    //占空比取绝对值
    pwm = (duty >= 0) ? (uint16)duty : (uint16)(-duty);

    gpio_set_level(ph_pin, dir);
    pwm_set_duty(en_pin, pwm);
}

//初始化电机驱动
//PE9/PE13 配成普通推挽输出当天方向脚用, 必须先拉低
//避免上电瞬间 PH 悬空导致电机乱转
void hw_motor_init(void)
{
    gpio_init(MOTOR_L_PH_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_PH_PIN, GPO, 0, GPO_PUSH_PULL);

    //两个 EN 都先给 0 占空比 = 刹车状态, 再启动 PWM
    pwm_init(MOTOR_L_EN_PIN, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_R_EN_PIN, MOTOR_PWM_FREQ, 0);
}

//设置左右轮输出
void hw_motor_set(int16 left, int16 right)
{
    motor_output(MOTOR_L_PH_PIN, MOTOR_L_EN_PIN, left,  MOTOR_L_INVERT);
    motor_output(MOTOR_R_PH_PIN, MOTOR_R_EN_PIN, right, MOTOR_R_INVERT);
}

//刹车
void hw_motor_stop(void)
{
    pwm_set_duty(MOTOR_L_EN_PIN, 0);
    pwm_set_duty(MOTOR_R_EN_PIN, 0);
}
