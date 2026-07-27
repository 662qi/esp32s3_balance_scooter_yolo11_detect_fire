#ifndef __PWM_H__
#define __PWM_H__

#include "driver/ledc.h"
#include "driver/mcpwm.h"

/* ------ 配置参数，可在此处直接更改 ------ */
#define PWM_DUTY    0               // 初始PWM占空比 ( 范围：[0, (2**duty_resolution)] )
#define PWM_A_PIN   GPIO_NUM_18      // PWMA输出IO口
#define PWM_B_PIN   GPIO_NUM_6       // PWMB输出IO口

#define PWM_FREQ    15000           // PWM频率：15k Hz (电机最佳静音频率，手册推荐)
#define MCPWM_UNIT  MCPWM_UNIT_0    // MCPWM单元
#define PWM_A_TIMER MCPWM_TIMER_0   // PWMA所用定时器
#define PWM_B_TIMER MCPWM_TIMER_1   // PWMA所用定时器
#define MCPWM_A_IO  MCPWM0A         // MCPWM0输出引脚(正向)
#define MCPWM_B_IO  MCPWM1A         // MCPWM1输出引脚(正向)
#define MCPWM_DIR   MCPWM_GEN_A     // MCPWM输出方向(正向)


void pwm_init(void);
void pwma_set_duty(float percent);
void pwmb_set_duty(float percent);

#endif
