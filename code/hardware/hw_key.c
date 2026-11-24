#include "hw_key.h"

//从引脚直接读到的是原始值, 但四个和原理图的8-3优先编码器对不上
//所以这里做一次方向修正, 让对上层的值跟原理图标注一致
//下标是引脚原始值, 元素是修正后的值
static const uint8 key_fix_table[8] = {0, 1, 6, 3, 5, 4, 2, 7};

//修正值到按键编号的映射
//下标是修正值, 元素是对应按键
static const key_id_enum fix_to_id[8] =
{
    KEY_ID_S2,              //0  独立按键2
    KEY_ID_MID,             //1  五向中
    KEY_ID_RIGHT,           //2  五向右
    KEY_ID_S1,              //3  独立按键1
    KEY_ID_UP,              //4  五向上
    KEY_ID_DOWN,            //5  五向下
    KEY_ID_LEFT,            //6  五向左
    KEY_ID_NONE,            //7  无按键
};

//初始化按键编码器输出引脚(3根线均为编码器推挽输出,不需要内部上拉)
void hw_key_init(void)
{
    gpio_init(KEY_A0_PIN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_A1_PIN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_A2_PIN, GPI, 0, GPI_FLOATING_IN);
}

//读取编码器输出(返回 0~7, 已按原理图的方向修正)
//A0为最低位, 拼成3位二进制数
uint8 hw_key_raw_read(void)
{
    uint8 raw = gpio_get_level(KEY_A0_PIN)
              | (gpio_get_level(KEY_A1_PIN) << 1)
              | (gpio_get_level(KEY_A2_PIN) << 2);

    return key_fix_table[raw];
}

//读取当前按下的按键
key_id_enum hw_key_get(void)
{
    return fix_to_id[hw_key_raw_read()];
}
