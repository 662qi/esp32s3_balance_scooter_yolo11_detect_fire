#ifndef __KEY_H__
#define __KEY_H__

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define KEY_PIN GPIO_NUM_8

void key_init(void);
uint8_t key_get(void);
uint8_t key_get_state(void);
void key_tick(void);

#endif
