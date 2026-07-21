#include "mpu6050.h"

/**
  *@brieMPU6050指定地址写
  *@param	reg_addr 寄存器地址
  *@param   data     数据
  *@retval	无
  */
void mpu6050_write_reg(uint8_t reg_addr, uint8_t data)
{
    // 定义命令链句柄
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_h, reg_addr, true);
    i2c_master_write_byte(cmd_h, data, true);
    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_1, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);
}

/**
  *@brief	MPU6050指定地址读
  *@param	reg_addr 寄存器地址
  *@retval	data     数据
  */
uint8_t mpu6050_read_reg(uint8_t reg_addr)
{
    uint8_t data;
    // 定义命令链句柄
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_h, reg_addr, true);

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd_h, &data, I2C_MASTER_NACK);

    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_1, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);

    return data;
}

/**
  *@brief	MPU6050从指定地址开始批量读取
  *@param   reg_addr  起始寄存器
  *@param	buf       接收缓冲区
  *@param	len       长度
  *@retval  无
  */
void mpu6050_read_buf(uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_h, reg_addr, true);

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_h, buf, len, I2C_MASTER_LAST_NACK); // 批量读取

    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_1, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);
}

/**
  *@brief   MPU6050初始化
  *@param	无
  *@retval	无
  */
void mpu6050_init(void)
{
    myi2c1_init();

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);    // 电源管理寄存器1：解除睡眠，陀螺仪X轴时钟
    mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00);    // 电源管理寄存器2：6轴均不待机

    mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x07);    // 采用率：1k Hz
    mpu6050_write_reg(MPU6050_CONFIG, 0x00);        // 配置低通滤波器：不滤波(滤波使得速度变慢)

    mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x08);   // 陀螺仪配置寄存器：+- 500 °/s
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x18);  // 加速度计配置寄存器：最大量程
}

/**
  *@brief   MPU6050批量读取加速度+陀螺仪6轴数据（高速版，一次I2C读14字节）
  *@param   *AccX   加速度X轴原始数据
  *@param   *AccY   加速度Y轴原始数据
  *@param   *AccZ   加速度Z轴原始数据
  *@param   *GyroX  陀螺仪X轴原始数据
  *@param   *GyroY  陀螺仪Y轴原始数据
  *@param   *GyroZ  陀螺仪Z轴原始数据
  *@retval  无
  */
void mpu6050_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                        int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    /* 低速版
    uint8_t DataH, DataL;

    DataH = mpu6050_read_reg(MPU6050_ACCEL_XOUT_H);
    DataL = mpu6050_read_reg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;
    
    DataH = mpu6050_read_reg(MPU6050_ACCEL_YOUT_H);
    DataL = mpu6050_read_reg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;
    
    DataH = mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H);
    DataL = mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;
    
    DataH = mpu6050_read_reg(MPU6050_GYRO_XOUT_H);
    DataL = mpu6050_read_reg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;
    
    DataH = mpu6050_read_reg(MPU6050_GYRO_YOUT_H);
    DataL = mpu6050_read_reg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;
    
    DataH = mpu6050_read_reg(MPU6050_GYRO_ZOUT_H);
    DataL = mpu6050_read_reg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
    */

    // 高速版
    uint8_t buf[14];

    mpu6050_read_buf(MPU6050_ACCEL_XOUT_H, buf, 14);

    // 解析加速度计数据
    *AccX = (int16_t)(buf[0]  << 8 | buf[1]);
    *AccY = (int16_t)(buf[2]  << 8 | buf[3]);
    *AccZ = (int16_t)(buf[4]  << 8 | buf[5]);

    // 解析陀螺仪数据（跳过温度寄存器）
    *GyroX = (int16_t)(buf[8]  << 8 | buf[9]);
    *GyroY = (int16_t)(buf[10] << 8 | buf[11]);
    *GyroZ = (int16_t)(buf[12] << 8 | buf[13]);
}
