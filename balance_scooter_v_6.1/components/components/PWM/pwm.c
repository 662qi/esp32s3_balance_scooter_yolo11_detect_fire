#include "pwm.h"

/**
  *@brief	MCPWM初始化
  *@param	无
  *@retval	无
  *@note    2026.5.4 修改为MCPWM
  */
void pwm_init(void)
{
    // 定义MCPWM并配置结构体
    mcpwm_config_t mcpwm_cfg = {
        .cmpr_a = PWM_DUTY,                             // 通道A初始占空比
        .cmpr_b = PWM_DUTY,                             // 通道B初始占空比
        .counter_mode = MCPWM_UP_COUNTER,               // 计数器模式：向上计数
        .duty_mode = MCPWM_DUTY_MODE_0,                 // 占空比模式：高电平有效(标准电机驱动模式)
        .frequency = PWM_FREQ,                          // PWM波形频率
    };

    // 初始化两个定时器
    mcpwm_init(MCPWM_UNIT, PWM_A_TIMER, &mcpwm_cfg);    // PWM_A
    mcpwm_init(MCPWM_UNIT, PWM_B_TIMER, &mcpwm_cfg);    // PWM_B

    // 绑定PWM输出引脚
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM_A_IO, PWM_A_PIN); // PWM_A
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM_B_IO, PWM_B_PIN); // PWM_A
}

/**
  *@brief	PWMA修改占空比
  *@param	percent 百分比 ( 范围：0 ~ 100 )
  *@retval	无
  */
void pwma_set_duty(float percent)
{
    if(percent > 100)   percent = 100;
    if(percent < 0)     percent = 0;

    mcpwm_set_duty(MCPWM_UNIT, PWM_A_TIMER, MCPWM_DIR, percent);
}

/**
  *@brief	PWMB修改占空比
  *@param	duty    占空比 ( 范围：[0, (2**duty_resolution)] )
  *@retval	无
  */
void pwmb_set_duty(float percent)
{
    if(percent > 100)   percent = 100;
    if(percent < 0)     percent = 0;
    mcpwm_set_duty(MCPWM_UNIT, PWM_B_TIMER, MCPWM_DIR, percent);
}
