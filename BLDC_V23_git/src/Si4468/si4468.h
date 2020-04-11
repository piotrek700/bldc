#ifndef SI4468_H_
#define SI4468_H_

#include "platform.h"

#define SI4468_CMD_ACK_VALUE		0xFF
#define SI4468_TX_BUFF_MAX_SIZE		64

#define SI4468_IRQ_CHECK			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define SI4468_CTS_CHECK			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

typedef struct si446x_reply_FIFO_INFO_map si446x_reply_FIFO_INFO_map;

typedef struct __attribute__((__packed__)){
	uint8_t cmd;
	uint8_t data[SI4468_TX_BUFF_MAX_SIZE];
}Si4468TxFrame;

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

void si4468_read_frr_bcd_cb(uint8_t *rx);

void si4468_init(void);

void si4468_test(void);

bool si4468_get_init_status(void);

#endif
