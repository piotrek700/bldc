#ifndef LPS22HH_TRAN_H_
#define LPS22HH_TRAN_H_

#include <devices/lps22hh_reg.h>
#include "Spi/spi.h"

//1----------------------------------------------------------------------------
static const uint8_t tx_who_am_i[2] = { LPS22HH_SPI_READ_MASK | LPS22HH_REG_WHO_AM_I };
static volatile uint8_t rx_who_am_i[2];

static const SpiTransactionRecord_t record_who_am_i = {
		.p_tx_buff = (uint8_t *) tx_who_am_i,
		.p_rx_buff = (uint8_t *) rx_who_am_i,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 2,
		.p_cb = lps22hh_check_who_am_i_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_ctrl1_init[2] = { LPS22HH_SPI_WRITE_MASK | LPS22HH_REG_CTRL_REG1, LPS22HH_CTRL_REG1_P_T_200HZ };

static const SpiTransactionRecord_t record_ctrl1_init = {
		.p_tx_buff = (uint8_t *) tx_ctrl1_init,
		.p_rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 2,
		.p_cb = 0
};

//3----------------------------------------------------------------------------
static const uint8_t tx_read_sensor[6] = { LPS22HH_SPI_READ_MASK | LPS22HH_REG_PRESSURE_OUT_XL };
static volatile uint8_t rx_read_sensor[6];

static const SpiTransactionRecord_t record_read_sensor = {
		.p_tx_buff = (uint8_t *) tx_read_sensor,
		.p_rx_buff = (uint8_t *) rx_read_sensor,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 6,
		.p_cb = lps22hh_read_sensor_cb
};

#endif
