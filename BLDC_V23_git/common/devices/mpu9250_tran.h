#ifndef MPU9250_TRAN_H_
#define MPU9250_TRAN_H_

#include <devices/mpu9250_reg.h>
#include "Spi/spi.h"

//1----------------------------------------------------------------------------
static const uint8_t tx_who_am_i[2] = { MPU9250_SPI_READ_MASK | MPU9250_REG_WHO_AM_I };
static volatile uint8_t rx_who_am_i[2];

static const SpiTransactionRecord record_who_am_i = {
		.tx_buff = (uint8_t *) tx_who_am_i,
		.rx_buff = (uint8_t *) rx_who_am_i,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = mpu9250_check_who_am_i_cb
};

//2----------------------------------------------------------------------------
static const uint8_t tx_read_sensor[15] = { MPU9250_SPI_READ_MASK | MPU9250_REG_ACCEL_XOUT_H };
static volatile uint8_t rx_read_sensor[15];

static const SpiTransactionRecord record_read_sensor = {
		.tx_buff = (uint8_t *) tx_read_sensor,
		.rx_buff = (uint8_t *) rx_read_sensor,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 15,
		.cb = mpu9250_read_sensor_cb
};

//3----------------------------------------------------------------------------
static const uint8_t tx_pwr_mgnt_1_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_PWR_MGMT_1, MPU9250_PWR_MGMT_1_CLKSEL_AUTO };

static const SpiTransactionRecord record_pwr_mgnt_1_init = {
		.tx_buff = (uint8_t *) tx_pwr_mgnt_1_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//4----------------------------------------------------------------------------
static const uint8_t tx_config_init[2] = {MPU9250_SPI_WRITE_MASK | MPU9250_REG_CONFIG, MPU9250_CONFIG_DLPF_CFG_184HZ};

static const SpiTransactionRecord record_config_init = {
		.tx_buff = (uint8_t *) tx_config_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//5----------------------------------------------------------------------------
static const uint8_t tx_smplrt_div_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_SMPLRT_DIV, 0 };

static const SpiTransactionRecord record_smplrt_div_init = {
		.tx_buff = (uint8_t *) tx_smplrt_div_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//6----------------------------------------------------------------------------
static const uint8_t tx_gyro_config_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_CONFIG_FCHOISE_B_184HZ | MPU9250_GYRO_CONFIG_GYRO_FS_SEL_2000DPS };

static const SpiTransactionRecord record_gyro_config_init = {
		.tx_buff = (uint8_t *) tx_gyro_config_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//7----------------------------------------------------------------------------
static const uint8_t tx_accel_config_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_8G };

static const SpiTransactionRecord record_accel_config_init = {
		.tx_buff = (uint8_t *) tx_accel_config_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//8----------------------------------------------------------------------------
static const uint8_t tx_accel_config_2_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_ACCEL_CONFIG_2, MPU9250_ACCEL_CONFIG_2_A_DLPF_CFG_184HZ };

static const SpiTransactionRecord record_accel_config_2_init = {
		.tx_buff = (uint8_t *) tx_accel_config_2_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//9----------------------------------------------------------------------------
static const uint8_t tx_user_ctrl_init[2] = { MPU9250_SPI_WRITE_MASK | MPU9250_REG_USER_CTRL, 0x10 };

static const SpiTransactionRecord record_user_ctrl_init = {
		.tx_buff = (uint8_t *) tx_user_ctrl_init,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};

//10----------------------------------------------------------------------------
static uint8_t tx_reg[2] = { 0 , 0};

static SpiTransactionRecord record_tx_reg = {
		.tx_buff = (uint8_t *) tx_reg,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_IMU,
		.data_length = 2,
		.cb = 0
};


#endif
