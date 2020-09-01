#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#include "platform.h"

#define IMU_OFFSET_COUNTER						1024*8

#define LSM6DSL_8G_SENSITIVITY					(0.244f/1000.0f)
#define LSM6DSL_2000DPS_SENSITIVITY				(70.0f/1000.0f)
#define LSM6DSL_TEMPERATURE_SENSITIVITY			(1.0f/256.0f)
#define LSM6DSL_TEMPERATURE_OFFSET_C			25.0f
#define LSM6DSL_1G_TO_MS2						9.80665f

typedef struct __attribute__((__packed__)){
	int16_t temperature;
	int16_t gyroscope[3];
	int16_t accelerometer[3];
}Lsm6dslDataOutput;

void lsm6dsl_init(void);

float * lsm6dsl_get_imu_acceleration(void);

float * lsm6dsl_get_angular_velocity(void);

float lsm6dsl_get_temperature_c(void);

void lsm6dsl_read_sensor(void);

void lsm6dsl_test(void);

bool lsm6dsl_get_init_status(void);

void lsm6dsl_check_who_am_i_cb(uint8_t *rx);

void lsm6dsl_read_sensor_cb(uint8_t *rx);

#endif
