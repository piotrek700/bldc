#ifndef MPU9250_H_
#define MPU9250_H_

#include "platform.h"

#define IMU_OFFSET_COUNTER						1024*4

#define MPU9250_8G_SENSITIVITY					(1.0f/4096.0f)
#define MPU9250_2000DPS_SENSITIVITY				(1.0f/16.4f)
#define MPU9250_TEMPERATURE_SENSITIVITY			(1.0f/333.87f)
#define MPU9250_TEMPERATURE_OFFSET_C			21.0f
#define MPU9250_1G_TO_MS2						9.80665f

typedef struct __attribute__((__packed__)){
	int16_t accelerometer[3];
	int16_t temperature;
	int16_t gyroscope[3];
}Mpu9250DataOutput;

void mpu9250_init(void);

float * mpu9250_get_imu_acceleration(void);

float * mpu9250_get_angular_velocity(void);

float mpu9250_get_temperature_c(void);

void mpu9250_read_sensor(void);

void mpu9250_test(void);

bool mpu9250_get_init_status(void);

void mpu9250_check_who_am_i_cb(uint8_t *rx);

void mpu9250_read_sensor_cb(uint8_t *rx);

#endif
