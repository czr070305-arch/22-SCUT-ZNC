#ifndef _HW_KEY_H_
#define _HW_KEY_H_

#include "zf_common_headfile.h"

//宏定义编码器输出引脚 A0/A1/A2 (对应 U15 的 9/7/6 脚)
#define KEY_A0_PIN      (D8)
#define KEY_A1_PIN      (B12)
#define KEY_A2_PIN      (B14)

//按键编号(对应7个物理按键)
typedef enum
{
    KEY_ID_NONE = 0,        //无按键
    KEY_ID_S2,              //独立按键2
    KEY_ID_MID,             //五向中
    KEY_ID_RIGHT,           //五向右
    KEY_ID_S1,              //独立按键1
    KEY_ID_UP,              //五向上
    KEY_ID_DOWN,            //五向下
    KEY_ID_LEFT,            //五向左
}key_id_enum;

void hw_key_init(void);

//读取编码器原始值, 返回屏幕显示用的0~7(已按原理图标注的方向修正)
uint8 hw_key_raw_read(void);

//读取当前按下的按键
key_id_enum hw_key_get(void);

#endif
