#include "tick.h"
#include "../Debug/debug.h"
#include "../Rybos/rybos.h"

CCMRAM_VARIABLE static volatile uint32_t tick_counter = 0;
static bool init_status = false;

void tick_test(void) {
	if ((RCC->CFGR & RCC_CFGR_SWS) != 0x08) {
		debug_error(CLOCK_SOURCE_ERROR);
	}

	if (SystemCoreClock != TICK_CPU_FREQUENCY_HZ) {
		debug_error(CLOCK_SPEED_ERROR);
	}

	if (!DEBUG_TEST_ENABLE) {
		return;
	}
}

uint32_t tick_get_time_ms(void) {
	return tick_counter;
}

void tick_delay_ms(uint32_t time) {
	time = tick_counter + time;
	while (time - tick_counter != 0);
}

void tick_init(void) {
	SystemCoreClockUpdate();

	*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
	*DWT_CYCCNT = 0;
	*DWT_CONTROL = *DWT_CONTROL | 1;

	SysTick_Config(SystemCoreClock / TICK_FREQUENCY_HZ);
	NVIC_SetPriority(SysTick_IRQn, 4);	//TODO replace to 1

	tick_test();

	init_status = true;
}
/*
uint32_t tick_get_clock_tick(void) {
	return *DWT_CYCCNT;
}*/

CCMRAM_FUCNTION void SysTick_Handler(void) {
	rybos_task_start_marker(MARKER_IRQ_TICK);
	tick_counter++;
	rybos_task_stop_marker(MARKER_IRQ_TICK);
}

bool tick_get_init_status(void) {
	return init_status;
}

