#include <sdk/log.h>
#include <stdio.h>
#include <sdk/cyclic_byte.h>
#include <sdk/rybos.h>
#include <sdk/debug.h>

static bool init_status = false;

CYCLIC_BUFFER_BYTE_DEF(tx_cyclic, true, LOG_TX_BUFFER_LENGTH);
CYCLIC_BUFFER_BYTE_DEF(rx_cyclic, true, LOG_RX_BUFFER_LENGTH);

void log_tx_state_mashine(void) {
	// ITM enabled and Port #0 enabled

	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while ((tx_cyclic.elements > 0) && (ITM->PORT[0].u32 != 0)) {
			uint8_t data;
			cyclic_byte_get((CyclicByteBuffer_t *) &tx_cyclic, &data);
			ITM->PORT[0].u8 = data;
		}
	}

	//if (tx_cyclic.elements == 0) {
		//rybos_task_enable(RYBOS_MARKER_TASK_LOGGER, false);
	//}
}

void log_tx_flush(void) {
	//Flush printf
	fflush(stdout);

	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (tx_cyclic.elements) {
			uint8_t data;
			cyclic_byte_get((CyclicByteBuffer_t *) &tx_cyclic, &data);
			ITM_SendChar(data);
		}
	}
}
void log_rx_state_mashine(void) {
	if (ITM_CheckChar()) {
		cyclic_byte_add((CyclicByteBuffer_t *) &rx_cyclic, ITM_ReceiveChar());
	}
}

void log_send_byte(uint8_t data) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		cyclic_byte_add((CyclicByteBuffer_t *) &tx_cyclic, data);

		//rybos_task_enable(RYBOS_MARKER_TASK_LOGGER, true);
	}
}

void log_send_string(uint8_t *p_string) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (*p_string) {
			cyclic_byte_add((CyclicByteBuffer_t *) &tx_cyclic, *p_string++);
		}
		//rybos_task_enable(RYBOS_MARKER_TASK_LOGGER, true);
	}
}

void log_send_string_len(uint8_t *p_string, uint32_t len) {
	// ITM enabled and Port #0 enabled
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << 0))) {
		while (len--) {
			cyclic_byte_add((CyclicByteBuffer_t *) &tx_cyclic, *p_string++);
		}
		//rybos_task_enable(RYBOS_MARKER_TASK_LOGGER, true);
	}
}

bool log_get_byte(uint8_t *p_data) {
	return cyclic_byte_get((CyclicByteBuffer_t *) &rx_cyclic, p_data);
}

void log_clear_rx_buff(void) {
	cyclic_byte_clear((CyclicByteBuffer_t *) &rx_cyclic);
}

bool log_get_init_status(void) {
	return init_status;
}

void log_init(void) {
	init_status = true;
}

