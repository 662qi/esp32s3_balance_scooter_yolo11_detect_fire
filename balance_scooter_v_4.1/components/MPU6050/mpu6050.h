#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "myi2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050_reg.h"

#define MPU6050_ADDR    0x68    // MPU6050地址

void mpu6050_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t mpu6050_read_reg(uint8_t reg_addr);
void mpu6050_init(void);
void mpu6050_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
