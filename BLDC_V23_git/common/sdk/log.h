#ifndef LOG_H_
#define LOG_H_

#include "platform.h"

#define LOG_TX_BUFFER_LENGTH	2048
#define LOG_RX_BUFFER_LENGTH	16

void log_tx_state_mashine(void);

void log_tx_flush(void);

void log_rx_state_mashine(void);

void log_send_byte(uint8_t data);

void log_send_string(uint8_t *p_string);

void log_send_string_len(uint8_t *p_string, uint32_t len);

bool log_get_byte(uint8_t *p_data);

void log_clear_rx_buff(void);

bool log_get_init_status(void);

void log_init(void);

#endif
