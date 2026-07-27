#ifndef __LED_H__
#define __LED_H__

#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_18

void LED_init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Toggle(void);

#endif
