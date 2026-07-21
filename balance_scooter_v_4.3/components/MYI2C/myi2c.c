#include "myi2c.h"

/**
  *@brief	I2C_0初始化
  *@param	无
  *@retval	无
  */
void myi2c0_init(void)
{
    // 配置时钟源，GPIO交换矩阵引脚，I2C参数
    i2c_config_t i2c_cfg = {
        .clk_flags = 0,                     // 时钟源：0(按通讯速度自动分配时钟源)
        .master.clk_speed = 400000,         // 通讯速率：100k 与 400k Hz
        .mode = I2C_MODE_MASTER,            // 模式：主机模式
        .scl_io_num = SCL0_IO_NUM,           // SCL引脚IO口
        .scl_pullup_en = GPIO_PULLUP_ENABLE,// 上拉电阻：开启
        .sda_io_num = SDA0_IO_NUM,           // SDA引脚IO口
        .sda_pullup_en = GPIO_PULLUP_ENABLE,// 上拉电阻：开启

        // .slave = ,                       // 配置为从机模式时使用
    };
    i2c_param_config(I2C_NUM_0, &i2c_cfg);

    // 向CPU注册I2C
    // I2C号，I2C模式，接收缓冲区大小(从机模式使用)，发送缓冲区大小(从机模式使用)，分配中断标志位
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

/**
  *@brief	I2C_1初始化
  *@param	无
  *@retval	无
  */
void myi2c1_init(void)
{
    // 配置时钟源，GPIO交换矩阵引脚，I2C参数
    i2c_config_t i2c_cfg = {
        .clk_flags = 0,                     // 时钟源：0(按通讯速度自动分配时钟源)
        .master.clk_speed = 400000,         // 通讯速率：100k 与 400k Hz
        .mode = I2C_MODE_MASTER,            // 模式：主机模式
        .scl_io_num = SCL1_IO_NUM,           // SCL引脚IO口
        .scl_pullup_en = GPIO_PULLUP_ENABLE,// 上拉电阻：开启
        .sda_io_num = SDA1_IO_NUM,           // SDA引脚IO口
        .sda_pullup_en = GPIO_PULLUP_ENABLE,// 上拉电阻：开启

        // .slave = ,                       // 配置为从机模式时使用
    };
    i2c_param_config(I2C_NUM_1, &i2c_cfg);

    // 向CPU注册I2C
    // I2C号，I2C模式，接收缓冲区大小(从机模式使用)，发送缓冲区大小(从机模式使用)，分配中断标志位
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
}
