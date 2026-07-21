#ifndef __MYI2C_H__
#define __MYI2C_H__

#include "driver/gpio.h"
#include "driver/i2c.h"

#define SCL0_IO_NUM  GPIO_NUM_13     // SCL0时钟线IO口
#define SDA0_IO_NUM  GPIO_NUM_14     // SDA0数据线IO口
#define SCL1_IO_NUM  GPIO_NUM_4      // SCL1时钟线IO口
#define SDA1_IO_NUM  GPIO_NUM_5      // SDA1数据线IO口

void myi2c0_init(void);
void myi2c1_init(void);

#endif
