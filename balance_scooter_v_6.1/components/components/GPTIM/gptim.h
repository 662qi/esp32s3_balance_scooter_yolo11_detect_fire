#ifndef __GPTIM_H__
#define __GPTIM_H__

#include <stdint.h>
#include "driver/gptimer.h"
extern gptimer_handle_t gptim;


bool Timer_Callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
void gptim_init(void);

#endif
