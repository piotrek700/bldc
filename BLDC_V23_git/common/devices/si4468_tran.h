#ifndef SI4468_TRAN_H_
#define SI4468_TRAN_H_

#include <devices/si4468_wds/Si446x/si446x_cmd.h>
#include <devices/si4468_wds/radio_config.h>
#include "Spi/spi.h"

//1----------------------------------------------------------------------------
static volatile uint8_t tx_init[16];

static volatile SpiTransactionRecord tran_init = {
		.tx_buff = (uint8_t *) tx_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 0,
		.cb = 0
};

//2----------------------------------------------------------------------------
static uint8_t tx_get_int_status[4] = { SI446X_CMD_ID_GET_INT_STATUS, 0, 0, 0 };

static SpiTransactionRecord record_get_int_status = {
		.tx_buff = (uint8_t *) tx_get_int_status,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 4,
		.cb = 0
};

///3----------------------------------------------------------------------------
static const uint8_t tx_read_frr_bcd[4] = { SI446X_CMD_ID_FRR_B_READ };
static volatile uint8_t rx_read_frr_bcd[4];

static const SpiTransactionRecord record_read_frr_bcd = {
		.tx_buff = (uint8_t *) tx_read_frr_bcd,
		.rx_buff = (uint8_t *) rx_read_frr_bcd,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 4,
		.cb = si4468_read_frr_bcd_cb
};

///4----------------------------------------------------------------------------
static const uint8_t tx_start_rx[8] = { SI446X_CMD_ID_START_RX, RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 0, 0, 0,
		SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
		SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
		SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX };

static const SpiTransactionRecord record_start_rx = {
		.tx_buff = (uint8_t *) tx_start_rx,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 8,
		.cb = 0
};

///5
static uint8_t tx_resp_int_status[10+1] = { SI446X_CMD_ID_READ_CMD_BUFF};
static uint8_t rx_resp_int_status[10+1];

static SpiTransactionRecord record_resp_int_status = {
		.tx_buff = (uint8_t *) tx_resp_int_status,
		.rx_buff = (uint8_t *) rx_resp_int_status,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 10,
		.cb = si4468_resp_int_status_cb
};

///6
static const uint8_t tx_read_frr_c[2] = { SI446X_CMD_ID_FRR_C_READ };
static volatile uint8_t rx_read_frr_c[2];

static const SpiTransactionRecord record_read_frr_c = {
		.tx_buff = (uint8_t *) tx_read_frr_c,
		.rx_buff = (uint8_t *) rx_read_frr_c,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = si4468_read_frr_c_cb
};

///7
static const uint8_t tx_chenge_state_to_rx[2] = { SI446X_CMD_ID_CHANGE_STATE, SI446X_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX};

static const SpiTransactionRecord record_chenge_state_to_rx = {
		.tx_buff = (uint8_t *) tx_chenge_state_to_rx,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = 0
};

#endif
