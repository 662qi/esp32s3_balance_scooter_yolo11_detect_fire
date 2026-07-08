#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

/* ------ 配置参数，可在此处直接更改 ------ */
#define ENC_A_A     GPIO_NUM_42      // E1A引脚
#define ENC_A_B     GPIO_NUM_2      // E1B引脚
#define ENC_B_A     GPIO_NUM_40     // E2A引脚
#define ENC_B_B     GPIO_NUM_41     // E2B引脚

#define ENC_PPR     1560            // 编码器每圈脉冲数(13线*4倍频*30减速比 = 1560)
#define PCNT_MAX    32767           // 计数器最大值
#define PCNT_MIN    -32767          // 计数器最小值
#define PCNT_INTR   0               // 中断优先级：默认

void encoder_init(void);
int32_t encoder_get_pulse(uint8_t dir);

#endif
