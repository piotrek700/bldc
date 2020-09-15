#ifndef TICK_H_
#define TICK_H_

#include "platform.h"
#include <sdk/utils.h>

#define TICK_FREQUENCY_HZ		1000
#define TICK_MAX_TIME_RANGE_S	0xFFFFFFFF/TICK_FREQUENCY_HZ
#define TICK_MAX_TIME_RANGE_H	TICK_MAX_TIME_RANGE_S/60/60
#define TICK_CPU_FREQUENCY_HZ	72000000

UNUSED_WARNING_DISABLE static volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
UNUSED_WARNING_DISABLE static volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
UNUSED_WARNING_DISABLE static volatile uint32_t *SCB_DEMCR = (uint32_t *) 0xE000EDFC;

void tick_test(void);

uint32_t tick_get_time_ms(void);

void tick_delay_ms(uint32_t time);

void tick_init(void);

void tick_clear_clock_tick(void);

bool tick_get_init_status(void);

static inline uint32_t tick_get_clock_tick(void) {
	return *DWT_CYCCNT;
}

#endif
