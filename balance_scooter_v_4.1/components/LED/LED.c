#include "LED.h"

/**
  *@brief	LED初始化
  *@param	无
  *@retval	无
  */
void LED_init(void)
{
    esp_err_t err;
    gpio_config_t gpio_cfg = {
        .intr_type = GPIO_INTR_DISABLE,         // GPIO中断
        .mode = GPIO_MODE_INPUT_OUTPUT,         // 输入输出模式
        .pin_bit_mask = 1ull << LED_PIN,        // 指定对应GPIO口
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // 下拉关闭
        .pull_up_en = GPIO_PULLUP_ENABLE,       // 上拉开启
    };
    err = gpio_config(&gpio_cfg);
    if(err != ESP_OK)
    {
        printf("LED_Init Error!\r\n");
    }
}

/**
  *@brief	LED开
  *@param	无
  *@retval	无
  */
void LED_ON(void)
{
    gpio_set_level(LED_PIN, 1);
}

/**
  *@brief	LED关
  *@param	无
  *@retval	无
  */
void LED_OFF(void)
{
    gpio_set_level(LED_PIN, 0);
}

/**
  *@brief	LED翻转
  *@param	无
  *@retval	无
  */
void LED_Toggle(void)
{
    /* 在输出模式下无法接收到GPIO口电平时可以这么写 */
    /*
    static uint8_t LED_State = 0;
    LED_State =! LED_State;
    gpio_set_level(LED_PIN, LED_State);
    */

    /* 只有在输入输出模式下才可接收到GPIO口电平，才可以如此写 */
    gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
}
