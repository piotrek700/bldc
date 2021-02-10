#ifndef LSM6DSL_TRANS_H_
#define LSM6DSL_TRANS_H_

#include <devices/lsm6dsl_reg.h>
#include "Spi/spi.h"

//1----------------------------------------------------------------------------
static const uint8_t tx_who_am_i[2] = { LSM6DSL_SPI_READ_MASK | LSM6DSL_REG_WHO_AM_I };
static volatile uint8_t rx_who_am_i[2];

static const SpiTransactionRecord_t record_who_am_i = {
		.tx_buff = (uint8_t *) tx_who_am_i,
		.rx_buff = (uint8_t *) rx_who_am_i,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = lsm6dsl_check_who_am_i_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_ctrl1_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL1_XL, LSM6DSL_CTRL1_XL_8G_833HZ | 0x01 | 0x02};

static const SpiTransactionRecord_t record_ctrl1_init = {
		.tx_buff = (uint8_t *) tx_ctrl1_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//3----------------------------------------------------------------------------
static const uint8_t tx_ctrl2_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL2_G, LSM6DSL_CTRL2_G_2000DPS_833HZ };

static const SpiTransactionRecord_t record_ctrl2_init = {
		.tx_buff = (uint8_t *) tx_ctrl2_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//4----------------------------------------------------------------------------
static const uint8_t tx_read_sensor[15] = { LSM6DSL_SPI_READ_MASK | LSM6DSL_REG_OUT_TEMP_L };
static volatile uint8_t rx_read_sensor[15];

static const SpiTransactionRecord_t record_read_sensor = {
		.tx_buff = (uint8_t *) tx_read_sensor,
		.rx_buff = (uint8_t *) rx_read_sensor,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 15,
		.cb = lsm6dsl_read_sensor_cb
};

//5----------------------------------------------------------------------------
static const uint8_t tx_ctrl4_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL4_C, LSM6DSL_CTRL4_C_LPF1_SEL_G_EN };

static const SpiTransactionRecord_t record_ctrl4_init = {
		.tx_buff = (uint8_t *) tx_ctrl4_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//6----------------------------------------------------------------------------
static const uint8_t tx_ctrl6_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL6_C, LSM6DSL_CTRL6_C_FTYPE_173HZ };

static const SpiTransactionRecord_t record_ctrl6_init = {
		.tx_buff = (uint8_t *) tx_ctrl6_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//7----------------------------------------------------------------------------
static const uint8_t tx_ctrl8_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL8_XL, LSM6DSL_CTRL8_XL_LPF2_XL_EN_ODR400 };

static const SpiTransactionRecord_t record_ctrl8_init = {
		.tx_buff = (uint8_t *) tx_ctrl8_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};


//8----------------------------------------------------------------------------
static const uint8_t tx_ctrl3_init[2] = { LSM6DSL_SPI_WRITE_MASK | LSM6DSL_REG_CTRL3_C, 0x04 | 0x40 };

static const SpiTransactionRecord_t record_ctrl3_init = {
		.tx_buff = (uint8_t *) tx_ctrl3_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

#endif
