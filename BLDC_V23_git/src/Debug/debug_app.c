#include "platform.h"
#include <stdio.h>
#include "../Bldc/bldc.h"
#include "../Led/led.h"
#include "../Log/log.h"
#include "utils.h"
#include "debug.h"

void debug_critical_error(DebugError error, uint8_t *file, int32_t line) {
	LED_RED_ON;											//TODO uncomment

	bldc_set_active_state(BLDC_STATE_STOP);

	printf("Critical error code: %u, file: %s, line: %d\n", (unsigned int)error, file, (int)line);
	log_tx_flush();
	while (1);
}

void debug_message_error(DebugError error, uint8_t *file, int32_t line) {
	if(error ==RADIO_RX_FRAME_SIZE_OUT_OF_RANGE){		//TODO remove
		//LED_RED_ON;									//TODO remove
	}

	if(error ==SI4468_PACKET_CRC_ERROR_IRQ){			//TODO remove
		//LED_RED_ON;									//TODO remove
		//LED_RED_OFF;
	}

	//LED_RED_ON; 										//TODO uncomment
	//LED_RED_OFF; 										//TODO uncomment

	//TODO add logger
	//printf("Error: %u, file: %s, line: %d\n", (unsigned int)error, file, (int)line);

	UNUSED(file);										//TODO remove
	UNUSED(line);										//TODO remove
}

