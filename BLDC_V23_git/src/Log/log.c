#include "log.h"

static bool init_status = false;

CYCLIC_BUFFER_BYTE_DEF(tx_cyclic, true, LOG_TX_BUFFER_LENGTH);
CYCLIC_BUFFER_BYTE_DEF(rx_cyclic, true, LOG_RX_BUFFER_LENGTH);

void log_tx_state_mashine(void) {
	// ITM enabled and Port #0 enabled

	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (tx_cyclic.elements > 0 && ITM->PORT[0].u32 != 0) {
			uint8_t data;
			cyclic_byte_get((CyclicByteBuffer *) &tx_cyclic, &data);
			ITM->PORT[0].u8 = data;
		}
	}

	if (tx_cyclic.elements == 0) {
		rybos_task_enable(MARKER_TASK_LOGGER, false);
	}
}

void log_tx_flush(void) {
	//Flush printf
	fflush(stdout);

	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (tx_cyclic.elements) {
			uint8_t data;
			cyclic_byte_get((CyclicByteBuffer *) &tx_cyclic, &data);
			ITM_SendChar(data);
		}
	}
}
void log_rx_state_mashine(void) {
	if (ITM_CheckChar()) {
		cyclic_byte_add((CyclicByteBuffer *) &rx_cyclic, ITM_ReceiveChar());
	}
}

void log_send_byte(uint8_t data) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		cyclic_byte_add((CyclicByteBuffer *) &tx_cyclic, data);
	}

	rybos_task_enable(MARKER_TASK_LOGGER, true);
}

void log_send_string(uint8_t *string) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (*string) {
			cyclic_byte_add((CyclicByteBuffer *) &tx_cyclic, *string++);
		}
	}

	rybos_task_enable(MARKER_TASK_LOGGER, true);
}

void log_send_string_len(uint8_t *string, uint32_t len) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (len--) {
			cyclic_byte_add((CyclicByteBuffer *) &tx_cyclic, *string++);
		}
	}

	rybos_task_enable(MARKER_TASK_LOGGER, true);
}

bool log_get_byte(uint8_t *data) {
	return cyclic_byte_get((CyclicByteBuffer *) &rx_cyclic, data);
}

void log_clear_rx_buff(void) {
	cyclic_byte_clear((CyclicByteBuffer *) &rx_cyclic);
}

bool log_get_init_status(void) {
	return init_status;
}

void log_init(void) {
	log_test();

	init_status = true;
}

void log_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

