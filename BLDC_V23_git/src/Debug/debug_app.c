#include "platform.h"
#include <stdio.h>
#include "Bldc/bldc.h"
#include "Led/led.h"
#include <sdk/log.h>
#include <sdk/utils.h>
#include <sdk/debug.h>

void debug_critical_error(DebugError_t error, uint8_t *p_file, int32_t line) {
	LED_RED_ON;

	bldc_set_active_state(BLDC_STATE_STOP);

	printf("Critical error: %u, %s, file: %s, line: %d\n", (unsigned int) error, debug_get_error_string(error), p_file, (int) line);
	log_tx_flush();

	while (1);
}

void debug_message_error(DebugError_t error, uint8_t *p_file, int32_t line) {
	//LED_RED_ON;
	//LED_RED_OFF;

	//printf("Error: %u, %s, file: %s, line: %d\n", (unsigned int) error, debug_get_error_string(error), p_file, (int) line);

	UNUSED(error);
	UNUSED(p_file);
	UNUSED(line);
}

