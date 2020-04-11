#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>

#define DEBUG_TEST_ENABLE	0

#define debug_error(error_code)					debug_error_handler(error_code, (uint8_t *)__FILE__, __LINE__)

typedef enum {
	CLOCK_SPEED_ERROR = 0,						//00
	CLOCK_SOURCE_ERROR,							//01
	DEBUG_ERROR_NOT_HANDLE,						//02
	FRAME_MESSAGE_OVERLENGTH,					//03
	FRAME_TYPE_NOT_SUPPORTED,					//04
	FRAME_BUFFER_OVERFLOW,						//05
	NMI_ERROR,									//06
	HARDFAULT_ERROR,							//07	+
	MEMMANAGE_ERROR,							//08
	BUSFAULT_ERROR,								//09
	USAGEFAULT_HANDLER,							//10
	SVC_ERROR,									//11
	DEBUGMON_ERROR,								//12
	PENDSV_ERROR,								//13
	DRV8301_SPI_READ_ERROR,						//14
	DMA_TX_OVERRUN_ERROR,						//15
	ROTOR_INJECTED_PHASE_ERROR,					//16
	BUZZER_SOUND_NOT_RECOGNISED,				//17
	SLAVE_SELECTED_NOT_SUPPORTED,				//18
	SPI_TRANSACTION_BUFFER_OVERFLOW,			//19	-
	PRESSURE_SENOR_NOT_DETECTED,				//20
	SERVO_POSITION_NOT_DEFINED,					//21
	SERVO_ANGLE_OUT_OF_RANGE,					//22
	IMU_SENOR_NOT_DETECTED,						//23
	DEBUG_RYBOS_TASK_LIST_LENGTH_ERROR,			//24
	RYBOS_TASK_MARKER_STACK_OVERLOW,			//25
	RYBOS_TASK_MARKER_ERROR,					//26
	DRV8301_READ_FRAME_FAULT,					//27
	ADC234_OVERRUN_ERROR,						//28
	ADC1_OVERRUN_ERROR,							//29
	BLDC_STATE_ERROR,							//30
	BLDC_PHASE_CNT_ERROR,						//31
	FRAME_CRC_ERROR,							//32
	SI4468_CMD_ERROR_IRQ,						//33	+
	SI4468_CMD_ERROR_RESP,						//34
	SI4468_PACKET_LOST_ERROR,					//35
	SI4468_RETRANSMITTION,						//36
	RADIO_SM_RX_CMD_NOT_SUPPORTED,				//37
	RADIO_SM_TX_CMD_NOT_SUPPORTED,				//38
	CYCLIC_PTR_OVERFLOW_CRITICAL,				//39
	SI4468_COMMAND_LENGTH_ERROR,				//40
	RADIO_CYCLIC_BUFFER_OVERFLOW,				//41	
	CYCLIC_BYTE_OVERFLOW_CRITICAL,				//42
	SI4468_RETRANSMITTION_LIMIT_REACHED,		//43	
	RADIO_TX_SM_RETRANSMITTION_LIMIT_REACHED,	//44
	RADIO_TX_SM_STATE_NOT_SUPPORTED,			//45
	RADIO_RX_SM_STATE_NOT_SUPPORTED,			//46
	RADIO_RX_SM_RETRANSMITTION_LIMIT_REACHED,	//47
	ADC_TIMEOUT_ERROR,							//48
	UART_FRAME_MESSAGE_OVERLENGTH,				//49
	RADIO_RX_FRAME_SIZE_OUT_OF_RANGE,			//50
	SI4468_CMD_ERROR_IRQ_INIT,					//51
	RADIO_SM_ERROR_CMD_NOT_SUPPORTED,			//52
	RC_CONTROL_CRC_ERROR,						//53	-
	SI4468_PACKET_CRC_ERROR_IRQ,				//54
	CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL,			//55
	CYCLIC_BUFFER_OVERFLOW_CRITICAL,			//56
	VIBRATOR_SOUND_NOT_RECOGNISED,				//57
	CYCLIC_PTR_OVERFLOW_NO_CRITICAL,			//58
	CYCLIC_BYTE_OVERFLOW_NO_CRITICAL,			//59
	ADC_OFFSET_NOT_DEFINE_FOR_THIS_DEVICE,		//60
	DISPLAY_TYPE_NOT_SUPORTTED,					//61
	PID_TYPE_FRAME_NOT_SUPPORTED,				//62
	FRAME_DECODING_STATE_MASHINE_NO_STATE,		//63
	NO_ERROR = 0xFF
} DebugError;

void debug_error_handler(DebugError error, uint8_t *file, int32_t line);

#endif
