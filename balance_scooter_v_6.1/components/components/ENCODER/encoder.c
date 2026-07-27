#include "encoder.h"

// 定义两个PCNT单元句柄
static pcnt_unit_handle_t pcnt_unitA;
static pcnt_unit_handle_t pcnt_unitB;

/**
  *@brief	单路编码器初始化
  *@param	无
  *@retval	无
  */
static pcnt_unit_handle_t encoder_init_single(int gpio_a, int gpio_b, bool dir)
{
    // 定义PCNT单元句柄
    pcnt_unit_handle_t pcnt_unit;
    // 定义PCNT通道句柄
    pcnt_channel_handle_t pcnt_chan;

    /* --- 设置计数范围 --- */
    pcnt_unit_config_t pcnt_unit_cfg = {
        .flags.accum_count = 1,                 // 开启计数器累加模式
        .high_limit = PCNT_MAX,                 // 计数器最大值
        .low_limit = PCNT_MIN,                  // 计数器最小值
        .intr_priority = PCNT_INTR,             // 中断优先级
    };
    // 创建PCNT单元
    pcnt_new_unit(&pcnt_unit_cfg, &pcnt_unit);

    /* --- 配置编码器AB相 --- */
    pcnt_chan_config_t pcnt_chan_cfg = {
        .edge_gpio_num = gpio_a,                // 配置A相引脚
        .level_gpio_num = gpio_b,               // 配置B相引脚

        .flags.invert_edge_input = 0,           // 是否翻转A相：否
        .flags.invert_level_input = dir,        // 是否翻转B相：计次方向(1翻转，0不翻转)
        .flags.io_loop_back = 0,                // 是否开启IO环回模式(内部虚拟信号)：否
        .flags.virt_edge_io_level = 0,          // 设置虚拟信号的初始电平：0
        .flags.virt_level_io_level = 0,         // 设置虚拟信号的初始电平：0
    };
    // 创建PCNT通道
    pcnt_new_channel(pcnt_unit, &pcnt_chan_cfg, &pcnt_chan);

    /* --- 正交解码动作 --- */
    // A相上升沿 +1
    // A相下降沿 -1
    pcnt_channel_set_edge_action(pcnt_chan, 
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    // B相高电平 保持计数方向
    // B相低电平 反转方向
    pcnt_channel_set_level_action(pcnt_chan, 
                                 PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                 PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* --- 启动编码器 --- */
    pcnt_unit_enable(pcnt_unit);        // 使能PCNT
    pcnt_unit_clear_count(pcnt_unit);   // 清零计数器
    pcnt_unit_start(pcnt_unit);         // 开始计数
    
    return pcnt_unit;                   // 返回单元句柄
}

/**
  *@brief	encoder初始化
  *@param	无
  *@retval	无
  */
void encoder_init(void)
{
    pcnt_unitA = encoder_init_single(ENC_A_A, ENC_A_B, 1);     // 初始化电机A的编码器(方向无问题)
    pcnt_unitB = encoder_init_single(ENC_B_A, ENC_B_B, 0);     // 初始化电机B的编码器(方向翻转一下)
}

/**
  *@brief	encoder读取脉冲
  *@param	dir     读取的电机
  *@retval	count   读取的数据
  */
int32_t encoder_get_pulse(uint8_t dir)
{
    int count;
    if(dir == 1)
    {
        pcnt_unit_get_count(pcnt_unitA, &count);        // 读取数据
        pcnt_unit_clear_count(pcnt_unitA);              // 清空数据
    }
    else if(dir == 2)
    {
        pcnt_unit_get_count(pcnt_unitB, &count);        // 读取数据
        pcnt_unit_clear_count(pcnt_unitB);              // 清空数据
    }

    return count;
}
