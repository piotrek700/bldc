#ifndef LSM6DSL_TRANS_H_
#define LSM6DSL_TRANS_H_

#include <Imu/lsm6dsl_reg.h>
#include "../Spi/spi.h"

//1----------------------------------------------------------------------------
static const uint8_t tx_who_am_i[2] = { LSM6DSL_SPI_READ_MASK | LSM6DSL_REG_WHO_AM_I };
static volatile uint8_t rx_who_am_i[2];

static const SpiTransactionRecord record_who_am_i = {
		.tx_buff = (uint8_t *) tx_who_am_i,
		.rx_buff = (uint8_t *) rx_who_am_i,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = lsm6dsl_check_who_am_i_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_ctrl1_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL1_XL, LSM6DSL_CTRL1_XL_8G_1660HZ };

static const SpiTransactionRecord record_ctrl1_init = {
		.tx_buff = (uint8_t *) tx_ctrl1_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//3----------------------------------------------------------------------------
static const uint8_t tx_ctrl2_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL2_G, LSM6DSL_CTRL2_G_2000DPS_1660HZ };

static const SpiTransactionRecord record_ctrl2_init = {
		.tx_buff = (uint8_t *) tx_ctrl2_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//4----------------------------------------------------------------------------
static const uint8_t tx_read_sensor[15] = { LSM6DSL_SPI_READ_MASK | LSM6DSL_REG_OUT_TEMP_L };
static volatile uint8_t rx_read_sensor[15];

static const SpiTransactionRecord record_read_sensor = {
		.tx_buff = (uint8_t *) tx_read_sensor,
		.rx_buff = (uint8_t *) rx_read_sensor,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 15,
		.cb = lsm6dsl_read_sensor_cb
};

#endif
