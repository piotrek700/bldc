#ifndef IMU_H_
#define IMU_H_

#include "platform.h"

#define IMU_OFFSET_COUNTER						1024

#define LSM6DSL_8G_SENSITIVITY					(0.244f/1000.0f)
#define LSM6DSL_2000DPS_SENSITIVITY				(70.0f/1000.0f)
#define LSM6DSL_TEMPERATURE_SENSITIVITY			(1.0f/256.0f)
#define LSM6DSL_TEMPERATURE_OFFSET_C			25.0f
#define LSM6DSL_1G_TO_MS2						9.80665f

typedef struct __attribute__((__packed__)){
	int16_t temperature;
	int16_t gyroscope[3];
	int16_t accelerometer[3];
}ImuDataOutput;

void imu_init(void);

float * imu_get_imu_acceleration(void);

float * imu_get_angular_velocity(void);

float imu_get_temperature_c(void);

void imu_read_sensor(void);

void imu_test(void);

bool imu_get_init_status(void);

void imu_check_who_am_i_cb(uint8_t *rx);

void imu_read_sensor_cb(uint8_t *rx);

#endif
