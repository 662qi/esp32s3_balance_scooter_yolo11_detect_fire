#include "gptim.h"

// 定义定时器句柄
gptimer_handle_t gptim;

void gptim_init(void)
{
    // 配置时钟源和计数器
    gptimer_config_t gptimer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,     // 配置定时器时钟源：默认(即APB_CLK)
        .direction = GPTIMER_COUNT_UP,          // 配置计数器的计数方向：向上计数(从0开始)

        .flags.intr_shared = 0,                 // 是否共享中断号：否
        .flags.allow_pd = 0,                    // 是否允许定时器所在的电源域进入低功耗(掉电)模式：否
        .flags.backup_before_sleep = 0,         // 与allow_pd重复，已被废弃

        .intr_priority = 0,                     // 配置中断优先级：0(系统自动配置)
        .resolution_hz = 1000000,               // 配置计数节拍(步长)：(1 / resolution_hz) seconds：1us
    };
    gptimer_new_timer(&gptimer_cfg, &gptim);

    // 配置比较器
    gptimer_alarm_config_t gptimer_alarm_cfg = {
        .alarm_count = 1000,                    // 设置比较器的目标值：1000(1ms)
        .flags.auto_reload_on_alarm = 1,        // 是否开启自动重装载：是
        .reload_count = 0,                      // 设置计数器的重置值：0(仅在开启自动重装载时有效)
    };
    gptimer_set_alarm_action(gptim, &gptimer_alarm_cfg);

    // 配置报警事件
    gptimer_event_callbacks_t gptimer_event_cbs = {
        .on_alarm = Timer_Callback,             // 用于配置中断程序
    };
    gptimer_register_event_callbacks(gptim, &gptimer_event_cbs, NULL);

    // 使能定时器
    gptimer_enable(gptim);

    // 开启定时器
    gptimer_start(gptim);
}
