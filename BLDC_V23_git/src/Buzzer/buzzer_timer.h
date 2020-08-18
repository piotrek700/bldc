#ifndef BUZZER_TIMER_H_
#define BUZZER_TIMER_H_

#include "platform.h"

void buzzer_timer_init(void);

void buzzer_timer_test(void);

bool buzzer_timer_get_init_status(void);

void buzzer_timer_set_frequency(uint16_t freq);

#endif
