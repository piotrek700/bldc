#ifndef RADIO_TRAN_H_
#define RADIO_TRAN_H_

//1----------------------------------------------------------------------------
static const uint8_t tx_read_frr_a[2] = { SI446X_CMD_ID_FRR_A_READ };
static volatile uint8_t rx_read_frr_a[2];

static const SpiTransactionRecord record_read_frr_a = {
		.tx_buff = (uint8_t *) tx_read_frr_a,
		.rx_buff = (uint8_t *) rx_read_frr_a,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = radio_read_frr_a_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_get_fifo_info[2] = { SI446X_CMD_ID_FIFO_INFO, 0 };
static volatile uint8_t rx_get_fifo_info[2];

static const SpiTransactionRecord record_get_fifo_info = {
		.tx_buff = (uint8_t *) tx_get_fifo_info,
		.rx_buff = (uint8_t *) rx_get_fifo_info,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = 0
};

//3----------------------------------------------------------------------------
static const uint8_t tx_resp_fifo_info[4] = { SI446X_CMD_ID_READ_CMD_BUFF };
static volatile uint8_t rx_resp_fifo_info[4];

static const SpiTransactionRecord record_resp_fifo_info = {
		.tx_buff = (uint8_t *) tx_resp_fifo_info,
		.rx_buff = (uint8_t *) rx_resp_fifo_info,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 4,
		.cb = radio_resp_fifo_info_cb
};

//4----------------------------------------------------------------------------
static const uint8_t tx_read_rx_fifo[64] = { SI446X_CMD_ID_READ_RX_FIFO };
static volatile uint8_t rx_read_rx_fifo[64];

static SpiTransactionRecord record_read_rx_fifo = {
		.tx_buff = (uint8_t *) tx_read_rx_fifo,
		.rx_buff = (uint8_t *) rx_read_rx_fifo,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 1,
		.cb = 0
};

//5----------------------------------------------------------------------------
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

//6----------------------------------------------------------------------------
static const uint8_t tx_reset_tx_fifo[2] = { SI446X_CMD_ID_FIFO_INFO, SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_BIT };
static volatile uint8_t rx_reset_tx_fifo[2];

static const SpiTransactionRecord record_reset_tx_fifo = {
		.tx_buff = (uint8_t *) tx_reset_tx_fifo,
		.rx_buff = (uint8_t *) rx_reset_tx_fifo,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = 0
};

//7----------------------------------------------------------------------------
static volatile Si4468TxFrame tx_fill_fifo = { .cmd = SI446X_CMD_ID_WRITE_TX_FIFO };
static volatile uint8_t rx_fill_fifo[64];

static volatile SpiTransactionRecord record_fill_fifo = {
		.tx_buff = (uint8_t *) &tx_fill_fifo,
		.rx_buff = (uint8_t *) rx_fill_fifo,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 1,
		.cb = 0
};

//8----------------------------------------------------------------------------
static volatile uint8_t tx_start_tx[7] = {
		SI446X_CMD_ID_START_TX,
		RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
		(SI446X_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_ENUM_RX << 4),
		0x00, //Data_length >> 8
		0x01, //Data_length
		0x00, //TX delay
		0x00  //Number of repeat
		};
static volatile uint8_t rx_start_tx[7];

static const SpiTransactionRecord record_start_tx = {
		.tx_buff = (uint8_t *) tx_start_tx,
		.rx_buff = (uint8_t *) rx_start_tx,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 7,
		.cb = 0
};

//9----------------------------------------------------------------------------
static volatile uint8_t tx_start_retransmit[7] = {
		SI446X_CMD_ID_START_TX,
		RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,
		(SI446X_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_ENUM_RX << 4) | SI446X_CMD_START_TX_ARG_CONDITION_RETRANSMIT_BIT,
		0x00, //Data_length >> 8
		0x01, //Data_length
		0x00, //TX delay
		0x00  //Number of repeat
		};
static volatile uint8_t rx_start_retransmit[7];

static const SpiTransactionRecord record_start_retransmit = {
		.tx_buff = (uint8_t *) tx_start_retransmit,
		.rx_buff = (uint8_t *) rx_start_retransmit,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 7,
		.cb = 0
};

//10----------------------------------------------------------------------------
static const uint8_t tx_reset_fifo[2] = { SI446X_CMD_ID_FIFO_INFO, SI446X_CMD_FIFO_INFO_ARG_FIFO_TX_MASK | SI446X_CMD_FIFO_INFO_ARG_FIFO_RX_MASK };
static volatile uint8_t rx_reset_fifo[2];

static const SpiTransactionRecord record_reset_fifo = {
		.tx_buff = (uint8_t *) tx_reset_fifo,
		.rx_buff = (uint8_t *) rx_reset_fifo,
		.slave = SPI_SLAVE_SELECT_RF,
		.data_length = 2,
		.cb = 0
};

#endif