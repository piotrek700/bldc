#include <devices/si4468.h>
#include <devices/si4468_tran.h>
#include <sdk/rybos.h>
#include <sdk/debug.h>
#include <sdk/tick.h>
#include "Spi/spi.h"
#include <devices/si4468_wds/compiler_defs.h>
#include <devices/si4468_wds/radio_config.h>
#include <devices/si4468_wds/Si446x/si446x_prop.h>
#include <devices/si4468_wds/Si446x/si446x_cmd.h>
#include "Si4468/si4468_drv.h"
#include <sdk/atomic.h>

static bool init_status = false;

static volatile bool frame_send = false;
static volatile bool cmd_ready = true;
static volatile bool rx_packet_pending = false;
static volatile bool cmd_error = false;
static volatile uint32_t nirq_ready = 0;

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
	if((!SI4468_DRV_CTS_CHECK) || cmd_ready ==true){
		return true;
	}else{
		return false;
	}
	//return cmd_ready;
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

void si4468_read_frr_bcd_cb(uint8_t *p_rx) {

	//	INVALID_PREAMBLE_PEND
	static uint32_t cmd_error_counter = 0;
	uint8_t ph_pend = p_rx[1];
	uint8_t modem_pend = p_rx[2];
	uint8_t chip_pend = p_rx[3];


	//tx_get_int_status[1] = ~rx[1];
	//	tx_get_int_status[2] = 0;
	//	tx_get_int_status[3] = ~rx[3];


	uint8_t buff[16];
	sprintf(buff, "IRQ %02X %02X %02X\n", p_rx[1], p_rx[2], p_rx[3]);
	log_send_string(buff);


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
			//return;
		}
	}

	if (ph_pend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_CRC_ERROR_PEND_BIT) {
		cmd_error = true;
		debug_error(SI4468_PACKET_CRC_ERROR_IRQ);
		rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
		//return;
	}

	//if(modem_pend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_INVALID_PREAMBLE_PEND_BIT){
	//	cmd_error = true;
	//	debug_error(SI4468_INVALID_PREAMBLE_PEND);
	//	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
	//	return;
	//}

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

#include <devices/si4468_wds/Si446x/si446x_cmd.h>

volatile uint32_t si4468_zero_error=0;
void si4468_resp_int_status_cb(uint8_t *p_rx) {
	if (p_rx[1] == SI4468_CMD_ACK_VALUE) {
		struct si446x_reply_GET_INT_STATUS_map *int_status = (struct si446x_reply_GET_INT_STATUS_map *) (p_rx + 2);

		uint8_t buff[4];
		buff[0] = 0;
		buff[1] = int_status->PH_PEND;
		buff[2] = int_status->MODEM_PEND;
		buff[3] = int_status->CHIP_PEND;
		si4468_read_frr_bcd_cb(buff);


		if(buff[1] ==0 && buff[2] ==0 && buff[3]==0){
			si4468_zero_error++;
		}

		si4468_set_cmd_ready(true);
	} else {
		debug_error(SI4468_CMD_ERROR_RESP);
	}
}

volatile bool ready_state = false;

bool si4468_get_ready_state(void){
	return ready_state;
}

void si4468_clear_ready_state(void){
	ready_state = false;
}

volatile uint32_t si4468_error_cnt = 0;

void si4468_read_frr_c_cb(uint8_t *p_rx){
	if(p_rx[1] != 0x08){
		if(rx_packet_pending == 0){
			ready_state = true;
			si4468_error_cnt++;
		}
	}

	uint8_t buff[8];
	sprintf(buff, "FFC %02X\n", p_rx[1]);
	log_send_string(buff);
}

void si4468_call_record_read_frr_c(void){
	log_send_string("RX?\n");
	spi_add_transaction((SpiTransactionRecord_t *) &record_read_frr_c);

}

void si4468_call_record_get_int_status(void){
	spi_add_transaction((SpiTransactionRecord_t *) &record_get_int_status);

}

void si4468_call_record_resp_int_status(void){
	spi_add_transaction((SpiTransactionRecord_t *) &record_resp_int_status);

}

void si4468_call_record_chenge_state_to_rx(void){
	spi_add_transaction((SpiTransactionRecord_t *) &record_chenge_state_to_rx);
}


void si4468_irq_process(void) {
	cmd_ready = false;

	//Read fast registers IRQ
	//spi_add_transaction((SpiTransactionRecord *) &record_read_frr_bcd);

	//Clear pending flags
	spi_add_transaction((SpiTransactionRecord_t *) &record_get_int_status);

}

bool si4468_get_nirq(void){
	return nirq_ready;
}

void si4468_decrease_nirq(){
	enter_critical();
	nirq_ready-- ;
	exit_critical();

}

void si4468_drv_nirq_cb(void) {
	log_send_string("NIRQ\n");

	//todo safe increment
	nirq_ready++;

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

void si4468_drv_cts_cb(void) {
	log_send_string("CTS\n");

	cmd_ready = true;

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

static void si4468_chip_init(void) {
	//Clear IRQ pending
	si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_get_int_status);

	//Clear IRQ pending
	si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_get_int_status);

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
			tran_init.p_tx_buff[i] = *init_data_ptr;
			init_data_ptr++;
		}
		si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &tran_init);

		//Check if not CMD error
		if (SI4468_DRV_IRQ_CHECK == 0) {
			//Read fast registers IRQ
			spi_add_transaction((SpiTransactionRecord_t *) &record_read_frr_bcd);

			//Clear pending flags
			si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_get_int_status);
		}
	}

	//Clear pending flags
	si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_get_int_status);

	//Set RX
	si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_start_rx);
	//Set RX
	si4468_drv_add_transaction_blocking((SpiTransactionRecord_t *) &record_start_rx);
}

void si4468_init(void) {
	si4468_drv_init();
	si4468_chip_init();
	si4468_drv_nvic_init();

	init_status = true;
}

bool si4468_get_init_status(void) {
	return init_status;
}
