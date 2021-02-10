#ifndef UART_H_
#define UART_H_

#include "platform.h"
#include <sdk/frame.h>

#define UART_FRAME_QUEUE_SIZE			16
#define UART_DMA_RX_BUFFER_LENGTH		256U
#define UART_FRAME_TX_BUFF_SIZE			256

typedef struct __attribute__((__packed__)) {
	uint32_t length;
	uint8_t tx_buff[UART_FRAME_TX_BUFF_SIZE];
} UartFrame_t;

void uart_reset_statistics(void);

uint32_t uart_get_transmitted_bytes(void);

uint32_t uart_get_received_bytes(void);

uint32_t uart_get_received_error_frames(void);

uint32_t uart_get_received_frames(void);

uint32_t uart_get_transmitted_frames(void);

void uart_increment_received_error_frame_cnt(void);

void uart_increment_reveived_frame_cnt(void);

void uart_init(void);

bool uart_get_init_status(void);

bool uart_get_byte_dma(uint8_t *p_data);

void uart_send_frame(FrameType_t type, uint8_t *p_frame, FrameParams_t params);

uint32_t uart_get_max_queue_depth(void);

void uart_send_scope_frame(FrameType_t type, uint8_t *p_frame, FrameParams_t params);

#endif
