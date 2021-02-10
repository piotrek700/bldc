#include <sdk/radio.h>
#include <sdk/radio_tran.h>
#include <string.h>
#include <sdk/debug.h>
#include <sdk/tick.h>
#include <sdk/buzzer.h>
#include <sdk/frame.h>
#include <sdk/cyclic.h>
#include <sdk/rybos.h>
#include <sdk/atomic.h>
#include "Uart/uart.h"

static bool init_status = false;

CYCLIC_BUFFER_DEF(radio_cyclic, true, RADIO_FRAME_QUEUE_SIZE, sizeof(RadioFrame_t));

static volatile RadioTxStateMachine_t tx_sm_state = RADIO_TX_SM_RESET_TX_FIFO;
static volatile RadioRxStateMachine_t rx_sm_state = RADIO_RX_SM_GET_FIFO_INFO;
static volatile RadioMasterStateMachine_t master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;
static volatile RadioSlaveStateMachine_t slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
static volatile bool wait_for_ack = false;

static volatile int32_t rssi_global = 0;
static volatile bool timeout = false;

//Statistics
static volatile int32_t max_rssi = RADIO_MIN_RSSI;
static volatile int32_t min_rssi = RADIO_MAX_RSSI;
static volatile uint32_t retransmition_cnt = 0;
static volatile uint32_t received_bytes = 0;
static volatile uint32_t transmiteded_bytes = 0;
static volatile uint32_t avarage_rssi_cnt = 0;
static volatile float avarage_rssi = 0;
static volatile uint32_t received_frame_cnt = 0;
static volatile uint32_t received_frame_total = 0;
static volatile uint32_t transmitted_frame_cnt = 0;

static void radio_reset_sm(void) {
	si4468_set_rx_packet_pending(false);
	si4468_set_frame_send(false);

	tx_sm_state = RADIO_TX_SM_RESET_TX_FIFO;
	rx_sm_state = RADIO_RX_SM_GET_FIFO_INFO;
	master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;
	slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
	wait_for_ack = false;
	timeout = false;
	//rybos_task_enable(MARKER_TASK_RF_TIMEOUT, false);
}

void radio_timeout_init(void) {
	timeout = false;
	rybos_task_enable_time(RYBOS_MARKER_TASK_RF_TIMEOUT, si4468_get_packet_sent_time() + RADIO_TX_TIMEOUT_CNT_MS, true);
}

void radio_timeout(void) {
	timeout = true;
	rybos_task_enable(RYBOS_MARKER_TASK_RF_TIMEOUT, false);
	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

static void radio_update_statistics(uint32_t frame_size, int32_t rssi) {
	if (rssi > max_rssi) {
		max_rssi = rssi;
	}

	if (rssi < min_rssi) {
		min_rssi = rssi;
	}

	avarage_rssi += rssi;
	avarage_rssi_cnt++;

	received_bytes += frame_size;
	received_frame_cnt++;
	received_frame_total++;
}

void radio_reset_statistics(void) {
	max_rssi = RADIO_MIN_RSSI;
	min_rssi = RADIO_MAX_RSSI;
	avarage_rssi = 0;
	avarage_rssi_cnt = 0;
	retransmition_cnt = 0;
	received_bytes = 0;
	transmiteded_bytes = 0;
	received_frame_cnt = 0;
	transmitted_frame_cnt = 0;
}

uint32_t radio_get_transmitted_frame(void) {
	return transmitted_frame_cnt;
}

uint32_t radio_get_received_frame(void) {
	return received_frame_cnt;
}

uint32_t radio_get_received_frame_total(void) {
	return received_frame_total;
}

uint32_t radio_get_received_bytes(void) {
	return received_bytes;
}

uint32_t radio_get_transmited_bytes(void) {
	return transmiteded_bytes;
}

int32_t radio_get_max_rssi(void) {
	return max_rssi;
}

int32_t radio_get_min_rssi(void) {
	return min_rssi;
}

int32_t radio_get_rssi(void) {
	return rssi_global;
}

uint32_t radio_get_retransmition_cnt(void) {
	return retransmition_cnt;
}

float radio_get_avarage_rssi(void) {
	return avarage_rssi / ((float) avarage_rssi_cnt);
}

static void radio_read_fifo_length(si446x_reply_FIFO_INFO_map_t *buff) {
	if(buff->RX_FIFO_COUNT<=RADIO_FRAME_TX_BUFF_SIZE){
		//Read package
		record_read_rx_fifo.data_length = buff->RX_FIFO_COUNT + 1;
	}else{
		si4468_set_cmd_error(true);
		//rybos_task_enable(MARKER_TASK_RF, true);
		debug_error(RADIO_RX_FRAME_SIZE_OUT_OF_RANGE);
	}
}

//1----------------------------------------------------------------------------
void radio_read_frr_a_cb(uint8_t *p_rx) {
	//Calculate RSSI
	rssi_global = ((p_rx[1] / 2) - 134);
}

//3----------------------------------------------------------------------------
void radio_resp_fifo_info_cb(uint8_t *p_rx) {
	if (p_rx[1] == SI4468_CMD_ACK_VALUE) {
		radio_read_fifo_length((si446x_reply_FIFO_INFO_map_t *) (p_rx + 2));
		si4468_set_cmd_ready(true);
	} else {
		debug_error(SI4468_CMD_ERROR_RESP);
	}

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

static bool radio_tx_sm(void) {
	//log_send_string("TX\n");

	if (si4468_get_cmd_ready()) {
		switch (tx_sm_state) {
		case RADIO_TX_SM_RESET_TX_FIFO:
			//log_send_string("TX1\n");
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord_t *) &record_reset_tx_fifo);
			tx_sm_state = RADIO_TX_SM_FILL_FIFO;
			break;

		case RADIO_TX_SM_FILL_FIFO:
			//log_send_string("TX2\n");
			si4468_set_frame_send(false);
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord_t *) &record_fill_fifo);
			spi_add_transaction((SpiTransactionRecord_t *) &record_start_tx);
			tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
			break;

		case RADIO_TX_SM_SEND_COMPLETE:
			//log_send_string("TX3\n");
			//Protect against frame update during the clean
			enter_critical();
			rybos_task_enable(RYBOS_MARKER_TASK_RF, false);

			if (si4468_get_frame_send()) {
				exit_critical();

				//log_send_string("TX4\n");
				//Statistics
				transmitted_frame_cnt++;
				transmiteded_bytes += tx_start_tx[4];
				tx_sm_state = RADIO_TX_SM_RESET_TX_FIFO;
				return true;
			}
			//log_send_string("TX5\n");
			exit_critical();
			break;

		default:
			debug_error(RADIO_SM_TX_CMD_NOT_SUPPORTED);
			break;
		}
	}
	return false;
}

static void radio_frame_received(RadioFrame_t *frame, uint32_t frame_size, int32_t rssi) {
	//Statistics
	radio_update_statistics(frame_size, rssi);

	if (frame->rx_tx_parameters & RADIO_PARAM_MORE_DATA) {
		FrameType_t type = frame->frame_type & FRAME_TYPE_MASK;
		uint8_t params = frame->frame_type & FRAME_PARAM_MASK;

		if (RADIO_MASTER) {
			if(params & FRAME_DESTINATION_MASTER_PC){
				frame_call_received_cb(type, params, frame->tx_buff );
			}

			//Always send to PC
			uart_send_frame(type, frame->tx_buff, params);

		} else {
			frame_call_received_cb(type, params, frame->tx_buff );
		}
	}

	//return;

	//TODO implement
	/*
	 printf("Frame RX: RSSI %d, L %d", (int) rssi, (int) frame->length);

	 if (frame->rx_tx_parameters & RADIO_PARAM_ACK_RESPONSE) {
	 printf(", ARSP");
	 }

	 if (frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE) {
	 printf(", AREQ");
	 }

	 if (frame->rx_tx_parameters & RADIO_PARAM_NOT_FINISHED) {
	 printf(", NF");
	 }

	 printf(", CNT %02X", frame->rx_tx_parameters & RADIO_PARAM_DATA_CNT);

	 if (frame->rx_tx_parameters & RADIO_PARAM_MORE_DATA) {
	 printf(", Data: %s", frame->tx_buff);
	 }

	 printf("\n");
	 */
}

static bool radio_rx_sm(void) {
	//log_send_string("RX\n");
	if (si4468_get_rx_packet_pending()) {
		//log_send_string("RX0\n");

		if (si4468_get_cmd_ready()) {
			switch (rx_sm_state) {
			case RADIO_RX_SM_GET_FIFO_INFO:
				//log_send_string("RX1\n");
				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord_t *) &record_get_fifo_info);
				rx_sm_state = RADIO_RX_SM_RESP_FIFO_INFO;
				break;

			case RADIO_RX_SM_RESP_FIFO_INFO:
				//log_send_string("RX2\n");
				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord_t *) &record_resp_fifo_info);
				rx_sm_state = RADIO_RX_SM_READ_RX_FIFO;
				break;

			case RADIO_RX_SM_READ_RX_FIFO:
				//log_send_string("RX3\n");
				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord_t *) &record_read_rx_fifo);
				spi_add_transaction((SpiTransactionRecord_t *) &record_read_frr_a);
				//TODO 130us avoid this stage, If TX buffer not empty go to TX state to transmit ACK
				spi_add_transaction((SpiTransactionRecord_t *) &record_start_rx);
				//TODO create a transaction which after TX go immediately to TX state to transmit ACK
				rx_sm_state = RADIO_RX_SM_RECEVIE_COMPLETE;
				break;

			case RADIO_RX_SM_RECEVIE_COMPLETE:
				//log_send_string("RX4\n");
				si4468_set_rx_packet_pending(false);
				rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
				rx_sm_state = RADIO_RX_SM_GET_FIFO_INFO;
				return true;
				break;

			default:
				debug_error(RADIO_SM_RX_CMD_NOT_SUPPORTED);
				break;
			}
		}
	} else {
		rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
	}

	return false;
}

static void radio_send_radio_frame(uint8_t ack_response, uint8_t ack_request, bool data) {
	static volatile uint32_t tx_cnt = 0;	//TODO is really volatile required
	RadioFrame_t *frame;

	if (data && cyclic_get((CyclicBuffer_t *) &radio_cyclic, (uint8_t **) &frame)) {
		//Prepare  transaction
		//| 1B CMD | 1B LEN | 1B RX_TX_PARAM | 1B FRAME_TYPE | 1B x Length = PEYLOAD|
		frame->rx_tx_parameters = tx_cnt & RADIO_PARAM_DATA_CNT;
		frame->rx_tx_parameters |= ack_response;
		frame->rx_tx_parameters |= ack_request;
		frame->rx_tx_parameters |= RADIO_PARAM_MORE_DATA;

		if (radio_cyclic.elements > 0) {
			frame->rx_tx_parameters |= RADIO_PARAM_NOT_FINISHED;
		}

		frame->length += 2;
		record_fill_fifo.data_length = frame->length + 2;									//+CMD + LEN + RX_TX_PARAM + FRAME_TYPE

		uint32_t i;
		//TODO implement memcpy_not_aligned
		for (i = 0; i < frame->length + 1U; i++) {
			record_fill_fifo.p_tx_buff[i + 1] = ((uint8_t *) frame)[i];
		}
		//memcpy(record_fill_fifo.tx_buff + 1, (uint8_t *) frame, frame->length + 1);			//+ LEN + RX_TX_PARAM + FRAME_TYPE

		//Copy packet length for transmission
		tx_start_tx[4] = frame->length + 1;													//+ LEN + RX_TX_PARAM + FRAME_TYPE
		//Copy packet length for retransmission
		tx_start_retransmit[4] = tx_start_tx[4];

		/*
		 printf("Frame TX: L %d", frame->length+1);

		 if (ack_response) {
		 printf(", ARSP");
		 }

		 if (ack_request) {
		 printf(", AREQ");
		 }

		 printf(", CNT %02X", tx_cnt & RADIO_PARAM_DATA_CNT);

		 if (frame->rx_tx_parameters & RADIO_PARAM_MORE_DATA) {
		 printf(", Data: %s", frame->tx_buff);
		 }

		 printf("\n");
		 */

		tx_cnt++;
	} else {
		//Prepare  transaction
		//| 1B CMD | 1B LEN | 1B RX_TX_PARAM |

		//RX_TX_PARAMETERS
		record_fill_fifo.p_tx_buff[2] = tx_cnt & RADIO_PARAM_DATA_CNT;
		record_fill_fifo.p_tx_buff[2] |= ack_response;
		record_fill_fifo.p_tx_buff[2] |= ack_request;
		//LEN
		record_fill_fifo.p_tx_buff[1] = 1;													//+CMD + LEN + RX_TX_PARAM

		record_fill_fifo.data_length = 3;

		//Copy packet length for transmission
		tx_start_tx[4] = 2;																	//LEN + RX_TX_PARAM
		//Copy packet length for retransmission
		tx_start_retransmit[4] = tx_start_tx[4];

		/*
		 printf("Frame TX: L %d", 1);

		 if (ack_response) {
		 printf(", ARSP");
		 }

		 if (ack_request) {
		 printf(", AREQ");
		 }

		 printf(", CNT %02X\n", tx_cnt & RADIO_PARAM_DATA_CNT);
		 */
		tx_cnt++;
	}

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);

}

static void radio_error_sm(void) {
	log_send_string("ERR\n");
	static RadioErrorStateMachine_t error_sm = RADIO_ERROR_SM_RESET_FIFO;

	if (si4468_get_cmd_ready()) {
		switch (error_sm) {
		case RADIO_ERROR_SM_RESET_FIFO:
			//Reset FIFO
			radio_reset_sm();
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord_t *) &record_reset_fifo);
			error_sm = RADIO_ERROR_SM_ENTER_RX;
			break;

		case RADIO_ERROR_SM_ENTER_RX:
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord_t *) &record_start_rx);
			error_sm = RADIO_ERROR_SM_ENTER_RX_COMPLETE;
			break;

		case RADIO_ERROR_SM_ENTER_RX_COMPLETE:
			si4468_set_cmd_error(false);
			rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
			error_sm = RADIO_ERROR_SM_RESET_FIFO;
			break;

		default:
			debug_error(RADIO_SM_ERROR_CMD_NOT_SUPPORTED);
			break;
		}
	} else {
		rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
	}
}


volatile bool unblock = false;

void radio_master_sm(void) {
	static int32_t retransmit_cnt = 0;

	//Bug fix - Si4468 enter ready state
	static uint32_t ready_sm = 0;

	if (si4468_get_ready_state()) {
		if (si4468_get_cmd_ready()) {
			switch (ready_sm) {
			case 0:
				si4468_set_cmd_ready(false);
				si4468_call_record_chenge_state_to_rx();
				ready_sm = 1;
				return;
				break;
			case 1:
				si4468_clear_ready_state();
				ready_sm = 0;
				break;
			default:
				break;
			}
		}
	}

	//IRQ Process
	static uint32_t irq_sm = 0;

	if (si4468_get_nirq()) {
		if (si4468_get_cmd_ready()) {
			switch (irq_sm) {
			case 0:
				si4468_set_cmd_ready(false);
				si4468_call_record_get_int_status();
				irq_sm = 1;
				return;
				break;
			case 1:
				si4468_set_cmd_ready(false);
				si4468_call_record_resp_int_status();
				irq_sm = 2;
				return;
				break;
			case 2:
				irq_sm = 0;
				si4468_decrease_nirq();
				break;
			default:
				break;
			}
		} else {
			return;
		}
	}

	//Prevent processing error data
	if (si4468_get_cmd_error()) {
		radio_error_sm();
		return;
	}

	switch (master_sm) {
	case RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME:
		log_send_string("MS1\n");
		if (radio_cyclic.elements > 0) {
			retransmit_cnt = 0;
			radio_send_radio_frame(0, RADIO_PARAM_ACK_REQUESTE, true);	//DATA+AR
			//First cycle of SM
			radio_tx_sm();
			master_sm = RADIO_MASTER_SM_TRANSMITTING;
		} else {

			if (si4468_get_rx_packet_pending()) {
				master_sm = RADIO_MASTER_SM_WAIT_FOR_ACK;
			}
			rybos_task_enable(RYBOS_MARKER_TASK_RF, false);
		}
		break;

	case RADIO_MASTER_SM_TRANSMITTING:
		log_send_string("MS2\n");

		if (radio_tx_sm()) {

			if (retransmit_cnt < 0) {
				//ACK not required
				master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;

				//NF
				if (retransmit_cnt == -2) {
					master_sm = RADIO_MASTER_SM_WAIT_FOR_ACK;
					radio_timeout_init();
					retransmit_cnt = RADIO_RETRANSMITION_LIMIT_CNT;
				}

			} else {
				master_sm = RADIO_MASTER_SM_WAIT_FOR_ACK;
				radio_timeout_init();
			}
		}
		break;

	case RADIO_MASTER_SM_WAIT_FOR_ACK:
		log_send_string("MS3\n");

		if (radio_rx_sm()) {
			RadioFrame_t * rx_frame = (RadioFrame_t *) (record_read_rx_fifo.p_rx_buff + 1);

			//Timeout disable
			if (rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_RESPONSE) {
				timeout = false;
				rybos_task_enable(RYBOS_MARKER_TASK_RF_TIMEOUT, false); //TODO required? radio_rx_sm should block
			}

			//Receiver handler
			radio_frame_received(rx_frame, record_read_rx_fifo.data_length - 1, rssi_global);

			//TODO optimize
			//ACK_NOT_REQUIRED		ACK_NOT_RECEIVED
			//nothing

			//ACK_NOT_REQUIRED		ACK_RECEIVED
			//check if data, if yes send DATA+RA or nothing

			//ACK_REQUIRED			ACK_NOT_RECEIVED
			//check if data, if yes send ACK+DATA+RA or ACK only

			//ACK_REQUIRED			ACK_RECEIVED
			//check if data, if yes send ACK+DATA+RA or ACK only

			//ACK_NOT_REQUIRED		ACK_RECEIVED
			if ((!(rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE)) && ((rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_RESPONSE))) {	//01
				if (radio_cyclic.elements > 0) {
					retransmit_cnt = 0;
					//ACK Response added- if the retransmission will be required ACK send i safe option
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//DATA+AR
					//First cycle of SM
					radio_tx_sm();
					master_sm = RADIO_MASTER_SM_TRANSMITTING;
				} else {
					//NF
					master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;

					if (rx_frame->rx_tx_parameters & RADIO_PARAM_NOT_FINISHED) {
						master_sm = RADIO_MASTER_SM_WAIT_FOR_ACK;
						radio_timeout_init();
						retransmit_cnt = RADIO_RETRANSMITION_LIMIT_CNT;
					}
					//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block

				}
			} else if (rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE) {																//1X
				if (radio_cyclic.elements > 0) {
					retransmit_cnt = 0;
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//A+DATA+AR
					radio_tx_sm();	//First cycle of sm
					master_sm = RADIO_MASTER_SM_TRANSMITTING;
				} else {
					retransmit_cnt = -1;
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, 0, false);																	//A
					radio_tx_sm();	//First cycle of sm
					master_sm = RADIO_MASTER_SM_TRANSMITTING;

					//NF
					if (rx_frame->rx_tx_parameters & RADIO_PARAM_NOT_FINISHED) {
						retransmit_cnt = -2;
					}
				}
			} else {																														//00
				//NF
				master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;

				if (rx_frame->rx_tx_parameters & RADIO_PARAM_NOT_FINISHED) {
					master_sm = RADIO_MASTER_SM_WAIT_FOR_ACK;
					radio_timeout_init();
					retransmit_cnt = RADIO_RETRANSMITION_LIMIT_CNT;
				}
				//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
			}

		} else if (si4468_get_rx_packet_pending()) {
			//Prevent exit SM when packet is processing
			//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
		} else {
			//Retransmit
			//TODO how to prevent task stop? 	rybos_task_enable(MARKER_TASK_RF, true);
			if (timeout) {
				//LED_RED_ON;
				//LED_RED_OFF;
				retransmit_cnt++;
				if (retransmit_cnt > RADIO_RETRANSMITION_LIMIT_CNT) {
					master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;
					//si4468_set_frame_send(false);
					//si4468_set_cmd_ready(false);
					debug_error(RADIO_TX_SM_RETRANSMITTION_LIMIT_REACHED);
				} else {

					if (si4468_get_cmd_ready()) {

						tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
						si4468_set_frame_send(false);
						si4468_set_cmd_ready(false);
						spi_add_transaction((SpiTransactionRecord_t *) &record_start_retransmit);
						//Statistics
						retransmition_cnt++;
						//transmiteded_bytes += tx_start_tx[4];
						master_sm = RADIO_MASTER_SM_TRANSMITTING;

					}
				}
			}
		}
		break;
	default:
		debug_error(RADIO_TX_SM_STATE_NOT_SUPPORTED);
		break;
	}
}


//TODO when enter to wit for frame check by ffr register if RX is enable - periodicaly?
void radio_slave_sm(void) {
	static int32_t retransmit_cnt = 0;

	//Bug fix - Si4468 enter ready state
	static uint32_t ready_sm = 0;

	if (si4468_get_ready_state()) {
		if (si4468_get_cmd_ready()) {
			switch (ready_sm) {
			case 0:
				si4468_set_cmd_ready(false);
				si4468_call_record_chenge_state_to_rx();
				ready_sm = 1;
				return;
				break;
			case 1:
				si4468_clear_ready_state();
				ready_sm = 0;
				break;
			default:
				break;
			}
		}
	}

	//IRQ Process
	static uint32_t irq_sm = 0;

	if (si4468_get_nirq()) {
		if (si4468_get_cmd_ready()) {
			switch (irq_sm) {
			case 0:
				si4468_set_cmd_ready(false);
				si4468_call_record_get_int_status();
				irq_sm = 1;
				return;
				break;
			case 1:
				si4468_set_cmd_ready(false);
				si4468_call_record_resp_int_status();
				irq_sm = 2;
				return;
				break;
			case 2:
				irq_sm = 0;
				si4468_decrease_nirq();
				break;
			default:
				break;
			}
		} else {
			return;
		}
	}

	//Prevent processing error data
	if (si4468_get_cmd_error()) {
		radio_error_sm();
		return;
	}

	switch (slave_sm) {
	case RADIO_SLAVE_SM_WAIT_FOR_FRAME:
		log_send_string("SM3\n");

		//TODO check periodically if stack here, if no frame rx read ffr to be sure that RX is enable - ugly but i not see any other option

		if (radio_rx_sm()) {
			RadioFrame_t * rx_frame = (RadioFrame_t *) (record_read_rx_fifo.p_rx_buff + 1);

			//Timeout disable
			if (rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_RESPONSE) {
				timeout = false;
				rybos_task_enable(RYBOS_MARKER_TASK_RF_TIMEOUT, false); //TODO required? radio_rx_sm should block
			}

			//Receiver handler
			radio_frame_received(rx_frame, record_read_rx_fifo.data_length - 1, rssi_global);

			if (wait_for_ack) {
				wait_for_ack = false;
				//TODO how to prevent task stop? 	rybos_task_enable(MARKER_TASK_RF, true);
				if (timeout) {
					//LED_RED_ON;	//TODO remove
					//LED_RED_OFF;	//TODO remove
					retransmit_cnt++;
					if (retransmit_cnt > RADIO_RETRANSMITION_LIMIT_CNT) {
						slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
						si4468_call_record_read_frr_c();

						//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
						debug_error(RADIO_RX_SM_RETRANSMITTION_LIMIT_REACHED);

					} else {
						tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
						si4468_set_frame_send(false);
						spi_add_transaction((SpiTransactionRecord_t *) &record_start_retransmit);
						//Statistics
						retransmition_cnt++;
						//transmiteded_bytes += tx_start_tx[4];
						si4468_set_cmd_ready(false);

						slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
						break;
					}
				}
			}

			//TODO optimize
			//ACK_NOT_REQUIRED		ACK_NOT_RECEIVED
			//nothing

			//ACK_NOT_REQUIRED		ACK_RECEIVED
			//check if data, if yes send DATA+RA or nothing

			//ACK_REQUIRED			ACK_NOT_RECEIVED
			//check if data, if yes send ACK+DATA+RA or ACK only

			//ACK_REQUIRED			ACK_RECEIVED
			//check if data, if yes send ACK+DATA+RA or ACK only

			//ACK_NOT_REQUIRED		ACK_RECEIVED
			if ((!(rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE)) && ((rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_RESPONSE))) {	//01
				if (radio_cyclic.elements > 0) {
					wait_for_ack = true;
					retransmit_cnt = 0;
					//ACK Response added- if the retransmission will be required ACK send i safe option
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//DATA+AR
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				} else {
					wait_for_ack = false;
					//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
					si4468_call_record_read_frr_c();
					slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;	//TODO remove
				}
			} else if (rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE) {																//1X
				if (radio_cyclic.elements > 0) {
					wait_for_ack = true;
					retransmit_cnt = 0;
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//A+DATA+AR
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				} else {
					wait_for_ack = false;
					retransmit_cnt = -1;
					radio_send_radio_frame(RADIO_PARAM_ACK_RESPONSE, 0, false);																	//A
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				}
			} else {																														//00
				//TODO should never enter here
				wait_for_ack = false;
				//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
				si4468_call_record_read_frr_c();
				slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;	//TODO remove
			}
		}

		break;

	case RADIO_SLAVE_SM_TRANSMITTING:
		log_send_string("SM4\n");

		if (radio_tx_sm()) {
			radio_timeout_init();
			si4468_call_record_read_frr_c();
			slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
		}
		break;

	default:
		debug_error(RADIO_RX_SM_STATE_NOT_SUPPORTED);
		break;
	}
}

void radio_send_frame(FrameType_t frame_type, uint8_t *p_frame, uint8_t params) {
	RadioFrame_t *frame_buff;
	enter_critical();

	frame_buff = (RadioFrame_t *) cyclic_get_to_add((CyclicBuffer_t *) &radio_cyclic);

	frame_buff->length = frame_get_type_length(frame_type);
	frame_buff->frame_type = frame_type | params;

	uint32_t i;
	for (i = 0; i < frame_buff->length; i++) {
		frame_buff->tx_buff[i] = p_frame[i];	//TODO memcopy
	}

	cyclic_move((CyclicBuffer_t *) &radio_cyclic);

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);

	exit_critical();

	if(RADIO_MASTER == 0){
		//Forward message via UART
		uart_send_frame(frame_type, p_frame, params);
	}
}

uint32_t radio_get_max_queue_depth(void) {
	return cyclic_get_max_elements((CyclicBuffer_t *) &radio_cyclic);
}

void radio_init(void) {
	si4468_init();

	init_status = true;
}

bool radio_get_init_status(void) {
	return init_status;
}
