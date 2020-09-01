#include "platform.h"
#include <stdio.h>
#include "../Bldc/bldc.h"
#include "../Led/led.h"
#include "../Log/log.h"
#include "utils.h"
#include "debug.h"

void debug_critical_error(DebugError error, uint8_t *file, int32_t line) {
	LED_RED_ON;

	bldc_set_active_state(BLDC_STATE_STOP);

	printf("Critical error: %u, %s, file: %s, line: %d\n", (unsigned int) error, debug_get_error_string(error), file, (int) line);
	log_tx_flush();

	while (1);
}

void debug_message_error(DebugError error, uint8_t *file, int32_t line) {
	//LED_RED_ON;
	//LED_RED_OFF;

	//printf("Error: %u, %s, file: %s, line: %d\n", (unsigned int) error, debug_get_error_string(error), file, (int) line);

	UNUSED(file);
	UNUSED(line);
}

