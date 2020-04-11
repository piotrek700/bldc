#ifndef UART_H_
#define UART_H_

#include "platform.h"

#define UART_FRAME_QUEUE_SIZE			16
#define UART_DMA_RX_BUFFER_LENGTH		256U
#define UART_FRAME_TX_BUFF_SIZE			256

typedef struct __attribute__((__packed__)) {
	uint32_t length;
	uint8_t tx_buff[UART_FRAME_TX_BUFF_SIZE];
} UartFrame;

void uart_reset_statistics(void);

uint32_t uart_get_transmitted_bytes(void);

uint32_t uart_get_received_bytes(void);

uint32_t uart_get_received_error_frames(void);

uint32_t uart_get_received_frames(void);

uint32_t uart_get_transmitted_frames(void);

void uart_increment_received_error_frame_cnt(void);

void uart_increment_reveived_frame_cnt(void);

void uart_init(void);

void uart_test(void);

bool uart_get_init_status(void);

bool uart_get_byte_dma(uint8_t *data);

void uart_send(uint8_t frame_type, uint8_t *frame, uint32_t frame_len);

uint32_t uart_get_max_queue_depth(void);

#endif
