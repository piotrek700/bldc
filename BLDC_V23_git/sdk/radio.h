#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include <stdbool.h>
#include <sdk/frame.h>

#define RADIO_FRAME_TX_BUFF_SIZE				61
#define RADIO_FRAME_QUEUE_SIZE					32
#define RADIO_RETRANSMITION_LIMIT_CNT			3		//1 transmission(not included) + 3 retransmission
#define RADIO_TX_TIMEOUT_CNT_MS					8		//TODO check minimum ~5ms = 50
#define RADIO_FRAME_PER_S_CONNECTION_LIMIT		10

#define RADIO_PARAM_ACK_RESPONSE				0x80
#define RADIO_PARAM_MORE_DATA					0x40
#define RADIO_PARAM_ACK_REQUESTE				0x20
#define RADIO_PARAM_ACK_NOT_REQUIRED			0x00
#define RADIO_PARAM_NOT_FINISHED				0x10
#define RADIO_PARAM_DATA_CNT					0x0F

#define RADIO_MAX_RSSI							32.0f
#define RADIO_MIN_RSSI							-256.0f

typedef enum {
	RADIO_MASTER_SM_WAIT_FOR_ADD_FRAME,
	RADIO_MASTER_SM_TRANSMITTING,
	RADIO_MASTER_SM_WAIT_FOR_ACK,
} RadioMasterStateMachine;

typedef enum {
	RADIO_RX_SM_GET_FIFO_INFO,
	RADIO_RX_SM_RESP_FIFO_INFO,
	RADIO_RX_SM_READ_RX_FIFO,
	RADIO_RX_SM_RECEVIE_COMPLETE
} RadioRxStateMachine;

typedef enum {
	RADIO_ERROR_SM_RESET_FIFO,
	RADIO_ERROR_SM_ENTER_RX,
	RADIO_ERROR_SM_ENTER_RX_COMPLETE
} RadioErrorStateMachine;

typedef enum {
	RADIO_TX_SM_RESET_TX_FIFO,
	RADIO_TX_SM_FILL_FIFO,
	RADIO_TX_SM_SEND_COMPLETE
} RadioTxStateMachine;

typedef enum {
	RADIO_SLAVE_SM_WAIT_FOR_FRAME,
	RADIO_SLAVE_SM_TRANSMITTING,
} RadioSlaveStateMachine;

typedef struct __attribute__((__packed__)) {
	uint8_t length;
	uint8_t rx_tx_parameters;
	uint8_t frame_type;
	uint8_t tx_buff[RADIO_FRAME_TX_BUFF_SIZE];
} RadioFrame;

void radio_timeout_init(void);

void radio_timeout(void);

void radio_reset_statistics(void);

uint32_t radio_get_transmitted_frame(void);

uint32_t radio_get_received_frame(void);

uint32_t radio_get_received_frame_total(void);

uint32_t radio_get_received_bytes(void);

uint32_t radio_get_transmited_bytes(void);

int32_t radio_get_max_rssi(void);

int32_t radio_get_min_rssi(void);

int32_t radio_get_rssi(void);

uint32_t radio_get_retransmition_cnt(void);

float radio_get_avarage_rssi(void);

void radio_read_frr_a_cb(uint8_t *rx);

void radio_resp_fifo_info_cb(uint8_t *rx);

void radio_master_sm(void);

void radio_slave_sm(void);

void radio_send_frame(FrameType frame_type, uint8_t *frame, uint8_t params);

uint32_t radio_get_max_queue_depth(void);

void radio_init(void);

void radio_test(void);

bool radio_get_init_status(void);

#endif
