#include <devices/lsm6dsl.h>
#include <devices/lsm6dsl_trans.h>
#include <sdk/debug.h>
#include "Spi/spi.h"

static bool init_status = false;
static volatile float acceleration[3] = { 0, 0, 0 };
static volatile float angular_velocity[3] = { 0, 0, 0 };
static volatile float temperature = 0;

//TODO remove

#define ACC_SIZE_3D 		500*6
int16_t acc_3d[ACC_SIZE_3D][3];
uint32_t acc_3d_cnt=0;

//1
void lsm6dsl_check_who_am_i_cb(uint8_t *rx) {
	if (rx[1] != LSM6DSL_WHO_AM_I_RESPONSE) {
		debug_error(IMU_SENOR_NOT_DETECTED);
	}
}

//4
void lsm6dsl_read_sensor_cb(uint8_t *rx) {
	Lsm6dslDataOutput *rx_imu = (Lsm6dslDataOutput *) (rx + 1);

	temperature = rx_imu->temperature * LSM6DSL_TEMPERATURE_SENSITIVITY + LSM6DSL_TEMPERATURE_OFFSET_C;
	acceleration[0] = rx_imu->accelerometer[0] * LSM6DSL_8G_SENSITIVITY * LSM6DSL_1G_TO_MS2;
	acceleration[1] = rx_imu->accelerometer[1] * LSM6DSL_8G_SENSITIVITY * LSM6DSL_1G_TO_MS2;
	acceleration[2] = rx_imu->accelerometer[2] * LSM6DSL_8G_SENSITIVITY * LSM6DSL_1G_TO_MS2;

	angular_velocity[0] = rx_imu->gyroscope[0] * LSM6DSL_2000DPS_SENSITIVITY;
	angular_velocity[1] = rx_imu->gyroscope[1] * LSM6DSL_2000DPS_SENSITIVITY;
	angular_velocity[2] = rx_imu->gyroscope[2] * LSM6DSL_2000DPS_SENSITIVITY;

	//TODO remove
	acc_3d[acc_3d_cnt][0]=rx_imu->accelerometer[0];
	acc_3d[acc_3d_cnt][1]=rx_imu->accelerometer[1];
	acc_3d[acc_3d_cnt][2]=rx_imu->accelerometer[2];

#include "Bldc/bldc.h"
	if(acc_3d_cnt != ACC_SIZE_3D-1){
		acc_3d_cnt++;
	}else if(acc_3d_cnt == ACC_SIZE_3D-1){
		if (bldc_get_active_state() == BLDC_STATE_FOC) {
			acc_3d_cnt = 0;
		}
		asm("nop");
	}
}

static void lsm6dsl_check_who_am_i(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_who_am_i);
}

static void lsm6dsl_lsm6dsl_init(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl1_init);
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl2_init);
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl3_init);

	//LPF enable
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl4_init);
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl6_init);
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl8_init);
}

void lsm6dsl_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	lsm6dsl_check_who_am_i();
	lsm6dsl_lsm6dsl_init();

	lsm6dsl_test();

	init_status = true;
}

float * lsm6dsl_get_imu_acceleration(void) {
	return (float *) acceleration;
}

float * lsm6dsl_get_angular_velocity(void) {
	return (float *) angular_velocity;
}

float lsm6dsl_get_temperature_c(void) {
	return temperature;
}

void lsm6dsl_read_sensor(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_read_sensor);
}

void lsm6dsl_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool lsm6dsl_get_init_status(void) {
	return init_status;
}
