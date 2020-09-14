#include "si4468.h"
#include "si4468_tran.h"
#include <sdk/rybos.h>
#include <sdk/debug.h>
#include <sdk/tick.h>
#include "Spi/spi.h"
#include "si4468_wds/bsp.h"
#include "si4468_wds/Si446x/si446x_cmd.h"

static bool init_status = false;

static volatile bool frame_send = false;
static volatile bool cmd_ready = true;
static volatile bool rx_packet_pending = false;
static volatile bool cmd_error = false;

static const uint8_t radio_conf_array[] = RADIO_CONFIGURATION_DATA_ARRAY;

static volatile uint32_t packet_sent_time = 0;
static volatile uint32_t packet_receive_time = 0;

uint32_t si4468_get_packet_sent_time(void) {
	return packet_sent_time;
}

uint32_t si4468_get_packet_receive_time(void) {
	return packet_receive_time;
}

bool si4468_get_cmd_error(void) {
	return cmd_error;
}

void si4468_set_cmd_error(bool cmd) {
	cmd_error = cmd;
}

bool si4468_get_frame_send(void) {
	return frame_send;
}

void si4468_set_frame_send(bool cmd) {
	frame_send = cmd;
}

bool si4468_get_cmd_ready(void) {
	return cmd_ready;
}

void si4468_set_cmd_ready(bool cmd) {
	cmd_ready = cmd;
	if (cmd == false) {
		rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
	}
}

void si4468_set_rx_packet_pending(bool cmd) {
	rx_packet_pending = cmd;
}

bool si4468_get_rx_packet_pending(void) {
	return rx_packet_pending;
}

//3----------------------------------------------------------------------------
void si4468_read_frr_bcd_cb(uint8_t *rx) {
	static uint32_t cmd_error_counter = 0;
	uint8_t ph_pend = rx[1];
	//uint8_t modem_pend = rx[2];
	uint8_t chip_pend = rx[3];

	if (chip_pend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_STATUS_CHIP_READY_BIT) {

	}

	if (chip_pend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_STATUS_CMD_ERROR_BIT) {
		if (init_status == false) {
			//Allow to have one command error - initialization power up command generate error
			if (cmd_error_counter == 0) {
				cmd_error_counter++;
			} else {
				debug_error(SI4468_CMD_ERROR_IRQ_INIT);
			}
		} else {
			cmd_error = true;
			debug_error(SI4468_CMD_ERROR_IRQ);
			rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
			return;
		}
	}

	if (ph_pend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_CRC_ERROR_PEND_BIT) {
		cmd_error = true;
		debug_error(SI4468_PACKET_CRC_ERROR_IRQ);
		rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
		return;
	}

	if (ph_pend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT) {
		if (rx_packet_pending) {
			debug_error(SI4468_PACKET_LOST_ERROR);
		}
		rx_packet_pending = true;
		packet_receive_time = tick_get_time_ms();
	}

	if (ph_pend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT) {
		frame_send = true;
		packet_sent_time = tick_get_time_ms();
	}

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

static void si4468_gpio_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void si4468_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(EXTI9_5_IRQn, 7);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(EXTI15_10_IRQn, 7);
}

static void si4468_exti_init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
}

void EXTI9_5_IRQHandler(void) {
	rybos_task_start_marker(RYBOS_MARKER_IRQ_SI4468_NIRQ);

	//IRQ
	EXTI_ClearITPendingBit(EXTI_Line5);

	//Read fast registers IRQ
	spi_add_transaction((SpiTransactionRecord *) &record_read_frr_bcd);

	//Clear pending flags
	spi_add_transaction((SpiTransactionRecord *) &record_get_int_status);
	cmd_ready = false;

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
	rybos_task_stop_marker(RYBOS_MARKER_IRQ_SI4468_NIRQ);
}

void EXTI15_10_IRQHandler(void) {
	rybos_task_start_marker(RYBOS_MARKER_IRQ_SI4468_CTS);

	//CTS
	EXTI_ClearITPendingBit(EXTI_Line10);

	cmd_ready = true;

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
	rybos_task_stop_marker(RYBOS_MARKER_IRQ_SI4468_CTS);
}

static void si4468_add_transaction_blocking(SpiTransactionRecord *record) {
	EXTI_ClearITPendingBit(EXTI_Line10);
	spi_add_transaction(record);
	while (EXTI_GetITStatus(EXTI_Line10) == 0);
	EXTI_ClearITPendingBit(EXTI_Line10);

	while (SI4468_CTS_CHECK == 0) {
		while (EXTI_GetITStatus(EXTI_Line10) == 0);
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

static void si4468_chip_init(void) {
	//Clear IRQ pending
	si4468_add_transaction_blocking((SpiTransactionRecord *) &record_get_int_status);

	//Clear IRQ pending
	si4468_add_transaction_blocking((SpiTransactionRecord *) &record_get_int_status);

	uint8_t const *init_data_ptr = radio_conf_array;
	uint32_t cmd_length;
	uint32_t i;

	while (*init_data_ptr != 0x00) {
		cmd_length = *init_data_ptr++;
		tran_init.data_length = cmd_length;

		if (cmd_length > 16u) {
			debug_error(SI4468_COMMAND_LENGTH_ERROR);
		}

		for (i = 0; i < cmd_length; i++) {
			tran_init.tx_buff[i] = *init_data_ptr;
			init_data_ptr++;
		}
		si4468_add_transaction_blocking((SpiTransactionRecord *) &tran_init);

		//Check if not CMD error
		if (SI4468_IRQ_CHECK == 0) {
			//Read fast registers IRQ
			spi_add_transaction((SpiTransactionRecord *) &record_read_frr_bcd);

			//Clear pending flags
			si4468_add_transaction_blocking((SpiTransactionRecord *) &record_get_int_status);
		}
	}

	//Clear pending flags
	si4468_add_transaction_blocking((SpiTransactionRecord *) &record_get_int_status);

	//Set RX
	si4468_add_transaction_blocking((SpiTransactionRecord *) &record_start_rx);
	//Set RX
	si4468_add_transaction_blocking((SpiTransactionRecord *) &record_start_rx);
}

void si4468_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	si4468_gpio_init();
	si4468_exti_init();
	si4468_chip_init();
	si4468_nvic_init();

	si4468_test();

	init_status = true;
}

void si4468_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool si4468_get_init_status(void) {
	return init_status;
}
