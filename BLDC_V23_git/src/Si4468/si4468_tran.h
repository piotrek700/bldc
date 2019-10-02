#ifndef SI4468_TRAN_H_
#define SI4468_TRAN_H_

//1----------------------------------------------------------------------------
static volatile uint8_t tx_init[16];
static volatile uint8_t rx_init[16];

static volatile SpiTransactionRecord tran_init = {
		.tx_buff = (uint8_t *) tx_init,
		.rx_buff = (uint8_t *) rx_init,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 0,
		.cb = 0
};

//2----------------------------------------------------------------------------
static const uint8_t tx_get_int_status[4] = { SI446X_CMD_ID_GET_INT_STATUS, 0, 0, 0 };
static volatile uint8_t rx_get_int_status[4];

static const SpiTransactionRecord record_get_int_status = {
		.tx_buff = (uint8_t *) tx_get_int_status,
		.rx_buff = (uint8_t *) rx_get_int_status,
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
static volatile uint8_t rx_start_rx[8];

static const SpiTransactionRecord record_start_rx = {
		.tx_buff = (uint8_t *) tx_start_rx,
		.rx_buff = (uint8_t *) rx_start_rx,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 8,
		.cb = 0
};

#endif
