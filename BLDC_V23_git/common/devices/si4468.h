#ifndef SI4468_H_
#define SI4468_H_

#include "platform.h"

#define SI4468_CMD_ACK_VALUE		0xFF
#define SI4468_TX_BUFF_MAX_SIZE		64

typedef struct __attribute__((__packed__)){
	uint8_t cmd;
	uint8_t data[SI4468_TX_BUFF_MAX_SIZE];
} Si4468TxFrame_t;

typedef struct si446x_reply_FIFO_INFO_map si446x_reply_FIFO_INFO_map_t;

uint32_t si4468_get_packet_sent_time(void);

uint32_t si4468_get_packet_receive_time(void);

bool si4468_get_cmd_error(void);

void si4468_set_cmd_error(bool cmd);

bool si4468_get_frame_send(void);

void si4468_set_frame_send(bool cmd);

bool si4468_get_cmd_ready(void);

void si4468_set_cmd_ready(bool cmd);

void si4468_set_rx_packet_pending(bool cmd);

bool si4468_get_rx_packet_pending(void);

void si4468_read_frr_bcd_cb(uint8_t *p_rx);

bool si4468_get_ready_state(void);

void si4468_clear_ready_state(void);

void si4468_read_frr_c_cb(uint8_t *p_rx);

void si4468_resp_int_status_cb(uint8_t *p_rx);

void si4468_call_record_read_frr_c(void);

void si4468_call_record_get_int_status(void);

void si4468_call_record_chenge_state_to_rx(void);

void si4468_call_record_resp_int_status(void);

void si4468_init(void);

bool si4468_get_init_status(void);

void si4468_irq_process(void);

bool si4468_get_nirq(void);

void si4468_decrease_nirq();

#endif
