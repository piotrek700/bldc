#include <devices/mpu9250.h>
#include <devices/mpu9250_tran.h>
#include <sdk/debug.h>
#include <sdk/utils.h>
#include <sdk/tick.h>
#include "Spi/spi.h"

static bool init_status = false;
static volatile float acceleration[3] = { 0, 0, 0 };
static volatile float angular_velocity[3] = { 0, 0, 0 };
static volatile float temperature = 0;

//1
void mpu9250_check_who_am_i_cb(uint8_t *rx) {
	if (rx[1] != MPU9250_WHO_AM_I_RESPONSE) {
		debug_error(IMU_SENOR_NOT_DETECTED);
	}
}

//4
void mpu9250_read_sensor_cb(uint8_t *rx) {
	Mpu9250DataOutput *rx_imu = (Mpu9250DataOutput *) (rx + 1);

	rx_imu->temperature = FAST_SWAP_INT16(rx_imu->temperature);

	rx_imu->accelerometer[0] = FAST_SWAP_INT16(rx_imu->accelerometer[0]);
	rx_imu->accelerometer[1] = FAST_SWAP_INT16(rx_imu->accelerometer[1]);
	rx_imu->accelerometer[2] = FAST_SWAP_INT16(rx_imu->accelerometer[2]);

	rx_imu->gyroscope[0] = FAST_SWAP_INT16(rx_imu->gyroscope[0]);
	rx_imu->gyroscope[1] = FAST_SWAP_INT16(rx_imu->gyroscope[1]);
	rx_imu->gyroscope[2] = FAST_SWAP_INT16(rx_imu->gyroscope[2]);

	temperature = rx_imu->temperature * MPU9250_TEMPERATURE_SENSITIVITY + MPU9250_TEMPERATURE_OFFSET_C;

	acceleration[0] = rx_imu->accelerometer[0] * MPU9250_8G_SENSITIVITY * MPU9250_1G_TO_MS2;
	acceleration[1] = rx_imu->accelerometer[1] * MPU9250_8G_SENSITIVITY * MPU9250_1G_TO_MS2;
	acceleration[2] = rx_imu->accelerometer[2] * MPU9250_8G_SENSITIVITY * MPU9250_1G_TO_MS2;

	angular_velocity[0] = rx_imu->gyroscope[0] * MPU9250_2000DPS_SENSITIVITY;
	angular_velocity[1] = rx_imu->gyroscope[1] * MPU9250_2000DPS_SENSITIVITY;
	angular_velocity[2] = rx_imu->gyroscope[2] * MPU9250_2000DPS_SENSITIVITY;
}

static void mpu9250_check_who_am_i(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_who_am_i);
}

static void mpu9250_mpu9250_init(void) {
	tick_delay_ms(200);
	spi_add_transaction((SpiTransactionRecord *) &record_user_ctrl_init);
	spi_add_transaction((SpiTransactionRecord *) &record_pwr_mgnt_1_init);
	//TODO check if required
	tick_delay_ms(200);
	spi_add_transaction((SpiTransactionRecord *) &record_config_init);
	spi_add_transaction((SpiTransactionRecord *) &record_smplrt_div_init);
	spi_add_transaction((SpiTransactionRecord *) &record_gyro_config_init);
	spi_add_transaction((SpiTransactionRecord *) &record_accel_config_init);
	spi_add_transaction((SpiTransactionRecord *) &record_accel_config_2_init);

/*
#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B

	uint8_t usr_init [][2]={
			{MPU9250_REG_PWR_MGMT_1,	0x01},
			{MPU9250_REG_PWR_MGMT_2,	0x00},
			{MPU9250_REG_CONFIG,		0x01},
			{MPU9250_REG_GYRO_CONFIG,	0x18},
			{MPU9250_REG_ACCEL_CONFIG,	0x08},
			{MPU9250_REG_ACCEL_CONFIG_2,0x09},
			{MPU9250_REG_INT_PIN_CFG,	0x22},
			{MPU9250_REG_USER_CTRL,		0x20},
			{MPU9250_REG_I2C_MST_CTRL,	0x0D},
			{MPU9250_REG_I2C_SLV0_ADDR,	AK8963_I2C_ADDR},
			{MPU9250_REG_I2C_SLV0_REG,	AK8963_CNTL2},
			{MPU9250_REG_I2C_SLV0_DO,	0x01},
			{MPU9250_REG_I2C_SLV0_CTRL,	0x81},
			{MPU9250_REG_I2C_SLV0_REG,	AK8963_CNTL1},
			{MPU9250_REG_I2C_SLV0_DO,	0x12},
			{MPU9250_REG_I2C_SLV0_CTRL,	0x81},
	};

	//tick_delay_ms(200);

	record_tx_reg.tx_buff[0] = MPU9250_REG_PWR_MGMT_1 | MPU9250_SPI_WRITE_MASK;
	record_tx_reg.tx_buff[1] = 0x80;;
	spi_add_transaction((SpiTransactionRecord *) &record_tx_reg);
	tick_delay_ms(200);

	for(uint32_t i=0; i<16; i++){
		record_tx_reg.tx_buff[0] = usr_init[i][0] | MPU9250_SPI_WRITE_MASK;
		record_tx_reg.tx_buff[1] = usr_init[i][1];
		spi_add_transaction((SpiTransactionRecord *) &record_tx_reg);
		tick_delay_ms(10);
	}
*/
}

void mpu9250_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	//TODO check if required
	//tick_delay_ms(200);
	mpu9250_check_who_am_i();
	mpu9250_mpu9250_init();

	mpu9250_test();

	init_status = true;
}

float * mpu9250_get_imu_acceleration(void) {
	return (float *) acceleration;
}

float * mpu9250_get_angular_velocity(void) {
	return (float *) angular_velocity;
}

float mpu9250_get_temperature_c(void) {
	return temperature;
}

void mpu9250_read_sensor(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_read_sensor);
}

void mpu9250_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool mpu9250_get_init_status(void) {
	return init_status;
}
