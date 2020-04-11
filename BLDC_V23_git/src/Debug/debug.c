#include "debug.h"
#include <stdio.h>
#include "../Bldc/bldc.h"
#include "../Led/led.h"
#include "../Log/log.h"
#include "utils.h"

static void debug_critical_error(DebugError error, uint8_t *file, int32_t line) {
	LED_RED_ON;											//TODO uncomment

	bldc_set_active_state(BLDC_STATE_STOP);

	printf("Critical error code: %u, file: %s, line: %d\n", (unsigned int)error, file, (int)line);
	log_tx_flush();
	while (1);
}

static void debug_message_error(DebugError error, uint8_t *file, int32_t line) {
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

void debug_error_handler(DebugError error, uint8_t *file, int32_t line){
	switch (error) {
		case CLOCK_SPEED_ERROR:
		case CLOCK_SOURCE_ERROR:
		case NMI_ERROR:
		case HARDFAULT_ERROR:
		case MEMMANAGE_ERROR:
		case BUSFAULT_ERROR:
		case USAGEFAULT_HANDLER:
		case SVC_ERROR:
		case DEBUGMON_ERROR:
		case PENDSV_ERROR:
		case DRV8301_SPI_READ_ERROR:
		case DMA_TX_OVERRUN_ERROR:
		case ROTOR_INJECTED_PHASE_ERROR:
		case BUZZER_SOUND_NOT_RECOGNISED:
		case SLAVE_SELECTED_NOT_SUPPORTED:
		case SPI_TRANSACTION_BUFFER_OVERFLOW:
		case PRESSURE_SENOR_NOT_DETECTED:
		case SERVO_POSITION_NOT_DEFINED:
		case SERVO_ANGLE_OUT_OF_RANGE:
		case IMU_SENOR_NOT_DETECTED:
		case DEBUG_RYBOS_TASK_LIST_LENGTH_ERROR:
		case RYBOS_TASK_MARKER_STACK_OVERLOW:
		case RYBOS_TASK_MARKER_ERROR:
		case DRV8301_READ_FRAME_FAULT:
		case ADC234_OVERRUN_ERROR:
		case ADC1_OVERRUN_ERROR:
		case BLDC_STATE_ERROR:
		case BLDC_PHASE_CNT_ERROR:
		case SI4468_CMD_ERROR_RESP:
		case RADIO_SM_RX_CMD_NOT_SUPPORTED:
		case RADIO_SM_TX_CMD_NOT_SUPPORTED:
		case UART_FRAME_MESSAGE_OVERLENGTH:
		case SI4468_COMMAND_LENGTH_ERROR:
		case RADIO_CYCLIC_BUFFER_OVERFLOW:
		case RADIO_TX_SM_STATE_NOT_SUPPORTED:
		case RADIO_RX_SM_STATE_NOT_SUPPORTED:
		case ADC_TIMEOUT_ERROR:
		case SI4468_CMD_ERROR_IRQ_INIT:
		case RADIO_SM_ERROR_CMD_NOT_SUPPORTED:
		case RC_CONTROL_CRC_ERROR:
		case CYCLIC_BUFFER_OVERFLOW_CRITICAL:
		case VIBRATOR_SOUND_NOT_RECOGNISED:
		case CYCLIC_PTR_OVERFLOW_CRITICAL:
		case CYCLIC_BYTE_OVERFLOW_CRITICAL:
		case ADC_OFFSET_NOT_DEFINE_FOR_THIS_DEVICE:
		case DISPLAY_TYPE_NOT_SUPORTTED:
		case PID_TYPE_FRAME_NOT_SUPPORTED:
		case FRAME_DECODING_STATE_MASHINE_NO_STATE:
		case DEBUG_ERROR_NOT_HANDLE:
		debug_critical_error(error, file, line);
		break;

		case SI4468_CMD_ERROR_IRQ:
		case SI4468_PACKET_LOST_ERROR:
		case FRAME_MESSAGE_OVERLENGTH:
		case FRAME_TYPE_NOT_SUPPORTED:
		case FRAME_BUFFER_OVERFLOW:
		case FRAME_CRC_ERROR:
		case SI4468_RETRANSMITTION:
		case SI4468_RETRANSMITTION_LIMIT_REACHED:
		case RADIO_TX_SM_RETRANSMITTION_LIMIT_REACHED:
		case RADIO_RX_SM_RETRANSMITTION_LIMIT_REACHED:
		case RADIO_RX_FRAME_SIZE_OUT_OF_RANGE:
		case SI4468_PACKET_CRC_ERROR_IRQ:
		case CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL:
		case CYCLIC_PTR_OVERFLOW_NO_CRITICAL:
		case CYCLIC_BYTE_OVERFLOW_NO_CRITICAL:
		debug_message_error(error, file, line);
		break;

		//No error do nothing
		case NO_ERROR:
		break;

		default:
		debug_error(DEBUG_ERROR_NOT_HANDLE);
		break;
	}
}

