#ifndef _HW_LED_H_
#define _HW_LED_H_

#include "zf_common_headfile.h"

//ºê¶¨ÒåLEDÒý½Å
#define LED5_PIN    (D1)
#define LED6_PIN    (A15)
#define LED7_PIN    (A12)
#define LED8_PIN    (A11)

void led_init(void);
void led_on(gpio_pin_enum pin);
void led_off(gpio_pin_enum pin);
void led_toggle(gpio_pin_enum pin);
#endif
