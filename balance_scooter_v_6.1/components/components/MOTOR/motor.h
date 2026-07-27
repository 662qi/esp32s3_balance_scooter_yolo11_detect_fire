#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pwm.h"
#include "driver/gpio.h"

#define AIN1_PIN    GPIO_NUM_16
#define AIN2_PIN    GPIO_NUM_17
#define BIN1_PIN    GPIO_NUM_15
#define BIN2_PIN    GPIO_NUM_7

void motor_init(void);
// 电机A
void motor_a_move(float speed);
// 电机B
void motor_b_move(float speed);

#endif
