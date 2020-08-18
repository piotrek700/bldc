#include "radio.h"
#include "radio_tran.h"
#include <string.h>
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../Buzzer/buzzer.h"
#include "../Frame/frame.h"
#include "../Cyclic/cyclic.h"
#include "../Rybos/rybos.h"
#include "../Atomic/atomic.h"

static bool init_status = false;

CYCLIC_BUFFER_DEF(radio_cyclic, true, RADIO_FRAME_QUEUE_SIZE, sizeof(RadioFrame));

static volatile RadioTxStateMachine tx_sm_state = RADIO_TX_SM_RESET_TX_FIFO;
static volatile RadioRxStateMachine rx_sm_state = RADIO_RX_SM_GET_FIFO_INFO;
static volatile RadioMasterStateMachine master_sm = RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME;
static volatile RadioSlaveStateMachine slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
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

static void radio_read_fifo_length(si446x_reply_FIFO_INFO_map *buff) {
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
void radio_read_frr_a_cb(uint8_t *rx) {
	//Calculate RSSI
	rssi_global = ((rx[1] / 2) - 134);
}

//3----------------------------------------------------------------------------
void radio_resp_fifo_info_cb(uint8_t *rx) {
	if (rx[1] == SI4468_CMD_ACK_VALUE) {
		radio_read_fifo_length((si446x_reply_FIFO_INFO_map *) (rx + 2));
		si4468_set_cmd_ready(true);
	} else {
		debug_error(SI4468_CMD_ERROR_RESP);
	}

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);
}

static bool radio_tx_sm(void) {
	if (si4468_get_cmd_ready()) {
		switch (tx_sm_state) {
		case RADIO_TX_SM_RESET_TX_FIFO:
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord *) &record_reset_tx_fifo);
			tx_sm_state = RADIO_TX_SM_FILL_FIFO;
			break;

		case RADIO_TX_SM_FILL_FIFO:
			spi_add_transaction((SpiTransactionRecord *) &record_fill_fifo);
			si4468_set_frame_send(false);
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord *) &record_start_tx);
			//radio_timeout_init();	//TODO ERROR
			tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
			break;

		case RADIO_TX_SM_SEND_COMPLETE:
			rybos_task_enable(RYBOS_MARKER_TASK_RF, false);

			//todo implement timeout
			//Waiting loop for TX complete
			if (si4468_get_frame_send()) {
				//Statistics
				transmitted_frame_cnt++;
				transmiteded_bytes += tx_start_tx[4];
				tx_sm_state = RADIO_TX_SM_RESET_TX_FIFO;

				//timeout = false;//TODO ERROR
				//rybos_task_enable(MARKER_TASK_RF_TIMEOUT, false); //TODO ERROR

				return true;
			}

			//if (timeout) {//TODO ERROR
				//LED_RED_ON;//TODO ERROR
				//LED_RED_OFF;//TODO ERROR
			//}//TODO ERROR

			break;

		default:
			debug_error(RADIO_SM_TX_CMD_NOT_SUPPORTED);
			break;
		}
	}
	return false;
}

static void radio_frame_received(RadioFrame *frame, uint32_t frame_size, int32_t rssi) {
	//Statistics
	radio_update_statistics(frame_size, rssi);

	if (frame->rx_tx_parameters & RADIO_PARAM_MORE_DATA) {
		FrameType type = frame->frame_type & FRAME_TYPE_MASK;
		uint8_t params = frame->frame_type & FRAME_PARAM_MASK;

		if (RADIO_MASTER) {
			if(params & FRAME_DESTINATION_MASTER_PC){
				frame_call_received_cb(type, frame->tx_buff, params);
			}

			//Always send to PC
			frame_uart_send(type, frame->tx_buff, params);

		} else {
			frame_call_received_cb(type, frame->tx_buff, params);
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
	if (si4468_get_rx_packet_pending()) {
		if (si4468_get_cmd_ready()) {
			switch (rx_sm_state) {
			case RADIO_RX_SM_GET_FIFO_INFO:
				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord *) &record_get_fifo_info);
				rx_sm_state = RADIO_RX_SM_RESP_FIFO_INFO;
				break;

			case RADIO_RX_SM_RESP_FIFO_INFO:
				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord *) &record_resp_fifo_info);
				rx_sm_state = RADIO_RX_SM_READ_RX_FIFO;
				break;

			case RADIO_RX_SM_READ_RX_FIFO:
				spi_add_transaction((SpiTransactionRecord *) &record_read_rx_fifo);
				spi_add_transaction((SpiTransactionRecord *) &record_read_frr_a);
				//TODO 130us avoid this stage, If TX buffer not empty go to TX state to transmit ACK

				si4468_set_cmd_ready(false);
				spi_add_transaction((SpiTransactionRecord *) &record_start_rx);
				//TODO create a transaction which after TX go immediately to TX state to transmit ACK
				rx_sm_state = RADIO_RX_SM_RECEVIE_COMPLETE;
				break;

			case RADIO_RX_SM_RECEVIE_COMPLETE:
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

static void radio_send_frame(uint8_t ack_response, uint8_t ack_request, bool data) {
	static volatile uint32_t tx_cnt = 0;	//TODO is really volatile required
	RadioFrame *frame;

	if (data && cyclic_get((CyclicBuffer *) &radio_cyclic, (uint8_t **) &frame)) {
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
			record_fill_fifo.tx_buff[i + 1] = ((uint8_t *) frame)[i];
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
		record_fill_fifo.tx_buff[2] = tx_cnt & RADIO_PARAM_DATA_CNT;
		record_fill_fifo.tx_buff[2] |= ack_response;
		record_fill_fifo.tx_buff[2] |= ack_request;
		//LEN
		record_fill_fifo.tx_buff[1] = 1;													//+CMD + LEN + RX_TX_PARAM

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
	static RadioErrorStateMachine error_sm = RADIO_ERROR_SM_RESET_FIFO;

	if (si4468_get_cmd_ready()) {
		switch (error_sm) {
		case RADIO_ERROR_SM_RESET_FIFO:
			//Reset FIFO
			radio_reset_sm();
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord *) &record_reset_fifo);
			error_sm = RADIO_ERROR_SM_ENTER_RX;
			break;

		case RADIO_ERROR_SM_ENTER_RX:
			si4468_set_cmd_ready(false);
			spi_add_transaction((SpiTransactionRecord *) &record_start_rx);
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

void radio_master_sm(void) {
	static int32_t retransmit_cnt = 0;

	//Prevent processing error data
	if(si4468_get_cmd_error()){
		radio_error_sm();
		return;
	}

	//TODO IRQ during tranmition protection - check
	//if (si4468_get_cmd_ready()==false){
	//	rybos_task_enable(MARKER_TASK_RF, false);
	//	return;
	//}

	switch (master_sm) {
	case RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME:

		if (radio_cyclic.elements > 0) {
			retransmit_cnt = 0;
			radio_send_frame(0, RADIO_PARAM_ACK_REQUESTE, true);	//DATA+AR
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
		if (radio_rx_sm()) {
			RadioFrame * rx_frame = (RadioFrame *) (record_read_rx_fifo.rx_buff + 1);

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
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//DATA+AR
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
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//A+DATA+AR
					radio_tx_sm();	//First cycle of sm
					master_sm = RADIO_MASTER_SM_TRANSMITTING;
				} else {
					retransmit_cnt = -1;
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, 0, false);																	//A
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
					debug_error(RADIO_TX_SM_RETRANSMITTION_LIMIT_REACHED);
				} else {
					tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
					si4468_set_frame_send(false);
					spi_add_transaction((SpiTransactionRecord *) &record_start_retransmit);
					//Statistics
					retransmition_cnt++;
					//transmiteded_bytes += tx_start_tx[4];
					si4468_set_cmd_ready(false);

					master_sm = RADIO_MASTER_SM_TRANSMITTING;
				}
			}
		}
		break;
	default:
		debug_error(RADIO_TX_SM_STATE_NOT_SUPPORTED);
		break;
	}
}

void radio_slave_sm(void) {
	static int32_t retransmit_cnt = 0;

	//Prevent processing error data
	if(si4468_get_cmd_error()){
		radio_error_sm();
		return;
	}

	//TODO IRQ during tranmition protection - check
	//if (si4468_get_cmd_ready()==false){
	//	rybos_task_enable(MARKER_TASK_RF, false);
	//	return;
	//}

	switch (slave_sm) {
	case RADIO_SLAVE_SM_WAIT_FOR_FRAME:
		if (radio_rx_sm()) {
			RadioFrame * rx_frame = (RadioFrame *) (record_read_rx_fifo.rx_buff + 1);

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
						//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
						debug_error(RADIO_RX_SM_RETRANSMITTION_LIMIT_REACHED);

					} else {
						tx_sm_state = RADIO_TX_SM_SEND_COMPLETE;
						si4468_set_frame_send(false);
						spi_add_transaction((SpiTransactionRecord *) &record_start_retransmit);
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
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//DATA+AR
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				} else {
					wait_for_ack = false;
					//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
					slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;	//TODO remove
				}
			} else if (rx_frame->rx_tx_parameters & RADIO_PARAM_ACK_REQUESTE) {																//1X
				if (radio_cyclic.elements > 0) {
					wait_for_ack = true;
					retransmit_cnt = 0;
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, RADIO_PARAM_ACK_REQUESTE, true);												//A+DATA+AR
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				} else {
					wait_for_ack = false;
					retransmit_cnt = -1;
					radio_send_frame(RADIO_PARAM_ACK_RESPONSE, 0, false);																	//A
					//First cycle of SM
					radio_tx_sm();
					slave_sm = RADIO_SLAVE_SM_TRANSMITTING;
				}
			} else {																														//00
				//TODO should never enter here
				wait_for_ack = false;
				//rybos_task_enable(MARKER_TASK_RF, false); //TODO required? radio_rx_sm should block
				slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;	//TODO remove
			}
		}

		break;

	case RADIO_SLAVE_SM_TRANSMITTING:
		if (radio_tx_sm()) {
			radio_timeout_init();
			slave_sm = RADIO_SLAVE_SM_WAIT_FOR_FRAME;
		}
		break;

	default:
		debug_error(RADIO_RX_SM_STATE_NOT_SUPPORTED);
		break;
	}
}

void radio_send_test_frame1(void) {		//TODO remove
	RadioFrame *frame;
	enter_critical();

	frame = (RadioFrame *)cyclic_get_to_add((CyclicBuffer *) &radio_cyclic);

	uint8_t tmp[] = "Piotrek700";
	frame->length = sizeof(tmp);
	frame->frame_type = 0x00;

	memcpy(frame->tx_buff, (uint8_t *) tmp, sizeof(tmp));
	cyclic_move((CyclicBuffer *) &radio_cyclic);

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);

	exit_critical();
}

void radio_send_test_frame2(void) { 	//TODO remove
	RadioFrame *frame;
	enter_critical();

	frame = (RadioFrame *)cyclic_get_to_add((CyclicBuffer *) &radio_cyclic);

	uint8_t tmp[] = "0 Ala ma kota a kot ma ale";
	static uint8_t value = '0';
	value++;
	if (value > '9') {
		value = '0';
	}
	tmp[0] = value;
	frame->length = sizeof(tmp);
	frame->frame_type = 0x09;

	memcpy(frame->tx_buff, (uint8_t *) tmp, sizeof(tmp));
	cyclic_move((CyclicBuffer *) &radio_cyclic);

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);

	exit_critical();
}

void radio_send(uint8_t frame_type, uint8_t *frame, uint32_t frame_len) {	//TODO remove frame length_ get in from frame dictionary
	RadioFrame *frame_buff;
	enter_critical();

	frame_buff = (RadioFrame *)cyclic_get_to_add((CyclicBuffer *) &radio_cyclic);

	frame_buff->length = frame_len;
	frame_buff->frame_type = frame_type;

	uint32_t i;
	for (i = 0; i < frame_len; i++) {
		frame_buff->tx_buff[i] = frame[i];	//TODO memcopy
	}

	//memcpy(frame_buff->tx_buff, frame, sizeof(frame_len));
	cyclic_move((CyclicBuffer *) &radio_cyclic);

	rybos_task_enable(RYBOS_MARKER_TASK_RF, true);

	exit_critical();
}

uint32_t radio_get_max_queue_depth(void) {
	return cyclic_get_max_elements((CyclicBuffer *) &radio_cyclic);
}

void radio_init(void) {
	si4468_init();

	radio_test();

	init_status = true;
}

void radio_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}

	//TODO Test
}

bool radio_get_init_status(void) {
	return init_status;
}
