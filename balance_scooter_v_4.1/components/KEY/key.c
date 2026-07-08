#include "key.h"

uint8_t key_num;

/**
  *@brief	按键初始化
  *@param	无
  *@retval	无
  */
void key_init(void)
{
    gpio_config_t gpio_cft = {
        .intr_type = GPIO_INTR_DISABLE,         // 无中断
        .mode = GPIO_MODE_INPUT,                // GPIO输入
        .pin_bit_mask = 1ull << KEY_PIN,        // 对应GPIO口
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // 下拉关闭
        .pull_up_en = GPIO_PULLUP_ENABLE,       // 上拉开启
    };
    gpio_config(&gpio_cft);
}

/**
  *@brief	按键获取(阻塞式)
  *@param	无
  *@retval	key_num 按键键码
  *//*
uint8_t key_get(void)
{
    uint8_t key_num = 0;

    if(gpio_get_level(KEY_PIN) == 0)
    {
        vTaskDelay(20);
        while(gpio_get_level(KEY_PIN) == 0);
        vTaskDelay(20);
        key_num = 1;
    }

    return key_num;
}
*/

/**
  *@brief	按键获取(非阻塞式)
  *@param	无
  *@retval	key_Num 按键键码
  */
uint8_t key_get(void)
{
    uint8_t temp;
    if(key_num)
    {
        temp = key_num;
        key_num = 0;
        return temp;
    }

    return 0;
}

/**
  *@brief	按键获取
  *@param	无
  *@retval	key_Num 按键键码
  */
uint8_t key_get_state(void)
{
    if(gpio_get_level(KEY_PIN) == 0)
    {
        return 1;
    }
    return 0;
}

/**
  *@brief	按键扫描(1ms)
  *@param	无
  *@retval	无
  */
void key_tick(void)
{
    static uint8_t Count;
    static uint8_t curr_state, prev_state;
    Count++;

    if(Count >= 20)
    {
        Count = 0;

        prev_state = curr_state;
        curr_state = key_get_state();

        if(curr_state == 0 && prev_state != 0)
        {
            key_num = prev_state;
        }
    }
}
