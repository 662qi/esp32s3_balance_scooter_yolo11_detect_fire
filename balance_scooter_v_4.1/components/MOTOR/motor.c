#include "motor.h"

/**
  *@brief	电机初始化
  *@param	无
  *@retval	无
  */
void motor_init(void)
{
    pwm_init();

    gpio_config_t gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << AIN1_PIN) |
                        (1Ull << AIN2_PIN) |
                        (1ULL << BIN1_PIN) |
                        (1Ull << BIN2_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&gpio_cfg);

    // 初始全停止
    gpio_set_level(AIN1_PIN, 0);
    gpio_set_level(AIN2_PIN, 0);
    gpio_set_level(BIN1_PIN, 0);
    gpio_set_level(BIN2_PIN, 0);
}

// 电机A
/**
  *@brief	电机A前进
  *@param	speed   速度
  *@retval	无
  */
void motor_a_forward(float speed)
{
    gpio_set_level(AIN1_PIN, 0); 
    gpio_set_level(AIN2_PIN, 1); 
    pwma_set_duty(speed);
}

/**
  *@brief	电机A后退
  *@param	speed   速度
  *@retval	无
  */
void motor_a_backward(float speed)
{
    gpio_set_level(AIN1_PIN, 1); 
    gpio_set_level(AIN2_PIN, 0); 
    pwma_set_duty(speed);
}

/**
  *@brief	电机A停止
  *@param	无
  *@retval	无
  */
void motor_a_stop(void)
{
    gpio_set_level(AIN1_PIN, 0); 
    gpio_set_level(AIN2_PIN, 0); 
    pwma_set_duty(0);
}

/**
  *@brief	电机A运动
  *@param	speed   速度
  *@retval	无
  */
void motor_a_move(float speed)
{
    if(speed > 0)
    {
        motor_a_forward(speed);
    }
    else if(speed < 0)
    {
        motor_a_backward(-speed);
    }
    else
    {
        motor_a_stop();
    }
}

// 电机B
/**
  *@brief	电机B前进
  *@param	speed   速度
  *@retval	无
  */
void motor_b_forward(float speed)
{
    gpio_set_level(BIN1_PIN, 0); 
    gpio_set_level(BIN2_PIN, 1); 
    pwmb_set_duty(speed);
}

/**
  *@brief	电机B后退
  *@param	speed   速度
  *@retval	无
  */
void motor_b_backward(float speed)
{
    gpio_set_level(BIN1_PIN, 1); 
    gpio_set_level(BIN2_PIN, 0); 
    pwmb_set_duty(speed);
}

/**
  *@brief	电机B停止
  *@param	无
  *@retval	无
  */
void motor_b_stop(void)
{
    gpio_set_level(BIN1_PIN, 0); 
    gpio_set_level(BIN2_PIN, 0); 
    pwmb_set_duty(0);
}

/**
  *@brief	电机B运动
  *@param	speed   速度
  *@retval	无
  */
void motor_b_move(float speed)
{
    if(speed > 0)
    {
        motor_b_forward(speed);
    }
    else if(speed < 0)
    {
        motor_b_backward(-speed);
    }
    else
    {
        motor_b_stop();
    }
}
