#include "hw_led.h"

//初始化LED5~8(高电平点亮)
void led_init(void)
{
    gpio_init(LED5_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LED6_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LED7_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LED8_PIN, GPO, 0, GPO_PUSH_PULL);
}

//点亮LED
void led_on(gpio_pin_enum pin)
{
    gpio_set_level(pin, 1);
}

//熄灭LED
void led_off(gpio_pin_enum pin)
{
    gpio_set_level(pin, 0);
}

//翻转LED
void led_toggle(gpio_pin_enum pin)
{
    gpio_toggle_level(pin);
}
