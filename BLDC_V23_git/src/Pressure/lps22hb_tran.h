#ifndef LPS22HB_TRAN_H_
#define LPS22HB_TRAN_H_

#include <Pressure/lps22hb_reg.h>
#include "../Spi/spi.h"

//1----------------------------------------------------------------------------
static const uint8_t tx_who_am_i[2] = { LPS22HB_SPI_READ_MASK | LPS22HB_REG_WHO_AM_I };
static volatile uint8_t rx_who_am_i[2];

static const SpiTransactionRecord record_who_am_i = {
		.tx_buff = (uint8_t *) tx_who_am_i,
		.rx_buff = (uint8_t *) rx_who_am_i,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 2,
		.cb = lps22hb_check_who_am_i_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_ctrl1_init[2] = { LPS22HB_SPI_WRITE_MASK | LPS22HB_REG_CTRL_REG1, LPS22HB_CTRL_REG1_P_T_75HZ };

static const SpiTransactionRecord record_ctrl1_init = {
		.tx_buff = (uint8_t *) tx_ctrl1_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 2,
		.cb = 0
};

//3----------------------------------------------------------------------------
static const uint8_t tx_read_sensor[6] = { LPS22HB_SPI_READ_MASK | LPS22HB_REG_PRESS_OUT_XL };
static volatile uint8_t rx_read_sensor[6];

static const SpiTransactionRecord record_read_sensor = {
		.tx_buff = (uint8_t *) tx_read_sensor,
		.rx_buff = (uint8_t *) rx_read_sensor,
		.slave = SPI_SLAVE_SELECT_PRESSURE,
		.data_length = 6,
		.cb = lps22hb_read_sensor_cb
};
//-----------------------------------------------------------------------------

#endif
