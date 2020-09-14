#ifndef DRV8301_TRAN_H_
#define DRV8301_TRAN_H_

#include "drv8301_reg.h"
#include "Spi/spi.h"
#include <sdk/utils.h>

//1----------------------------------------------------------------------------
static const uint16_t tx_stat_reg_1[1] = { SWAP_UINT16(DRV8301_SPI_READ | DRV8301_ADDR0_SR1) };
static volatile uint16_t rx_stat_reg_1[1];

static const SpiTransactionRecord read_stat_reg1 = {
		.tx_buff = (uint8_t *) tx_stat_reg_1,
		.rx_buff = (uint8_t *) rx_stat_reg_1,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_status_register2_cb
};

//2----------------------------------------------------------------------------
static const uint16_t tx_stat_reg_2[1] = { SWAP_UINT16(DRV8301_SPI_READ | DRV8301_ADDR1_SR2) };
static volatile uint16_t rx_stat_reg_2[1];

static const SpiTransactionRecord read_stat_reg2 = {
		.tx_buff = (uint8_t *) tx_stat_reg_2,
		.rx_buff = (uint8_t *) rx_stat_reg_2,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_status_register1_cb
};

//3----------------------------------------------------------------------------
static const uint16_t tx_ctrl_reg_1[1] = { SWAP_UINT16(DRV8301_SPI_WRITE| DRV8301_ADDR2_CR1 | DRV8301_CR1_SETTINGS) };
static volatile uint16_t rx_ctrl_reg_1[1];

static const SpiTransactionRecord write_ctrl_reg1 = {
		.tx_buff = (uint8_t *) tx_ctrl_reg_1,
		.rx_buff = (uint8_t *) rx_ctrl_reg_1,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_control_register1_cb
};

//4----------------------------------------------------------------------------
static const uint16_t tx_ctrl_reg_2[1] = { SWAP_UINT16(DRV8301_SPI_WRITE|DRV8301_ADDR3_CR2| DRV8301_CR2_SETTINGS) };
static volatile uint16_t rx_ctrl_reg_2[1];

static const SpiTransactionRecord write_ctrl_reg2 = {
		.tx_buff = (uint8_t *) tx_ctrl_reg_2,
		.rx_buff = (uint8_t *) rx_ctrl_reg_2,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_control_register2_cb
};

//5----------------------------------------------------------------------------
static const uint16_t tx_clear[1];

static const SpiTransactionRecord clear_trans = {
		.tx_buff = (uint8_t *) tx_clear,
		.rx_buff = 0,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = 0
};

//6----------------------------------------------------------------------------
static const uint16_t tx_calib_enable[1] = { SWAP_UINT16(
		DRV8301_SPI_WRITE|DRV8301_ADDR3_CR2| DRV8301_CR2_SETTINGS | DRV8301_CR2_DC_CAL_CH1_DISCONNECT|DRV8301_CR2_DC_CAL_CH2_DISCONNECT) };
static volatile uint16_t rx_calib_enable[1];

static const SpiTransactionRecord write_calib_enable = {
		.tx_buff = (uint8_t *) tx_calib_enable,
		.rx_buff = (uint8_t *) rx_calib_enable,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_calib_enable_cb
};

//7----------------------------------------------------------------------------
static const uint16_t tx_calib_disable[1] = { SWAP_UINT16(DRV8301_SPI_WRITE|DRV8301_ADDR3_CR2| DRV8301_CR2_SETTINGS) };
static volatile uint16_t rx_calib_disable[1];

static const SpiTransactionRecord write_calib_disable = {
		.tx_buff = (uint8_t *) tx_calib_disable,
		.rx_buff = (uint8_t *) rx_calib_disable,
		.slave = SPI_SLAVE_SELECT_BLDC,
		.data_length = 2,
		.cb = drv8301_calib_disable_cb
};

#endif
