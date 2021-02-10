#ifndef BUZZER_DRV_H_
#define BUZZER_DRV_H_

#include "platform.h"

void buzzer_drv_init(void);

bool buzzer_drv_get_init_status(void);

void buzzer_drv_set_frequency(uint16_t freq);

#endif
