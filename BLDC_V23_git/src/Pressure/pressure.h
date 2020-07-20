#ifndef PRESSURE_H_
#define PRESSURE_H_

#include "platform.h"

#define PRESSURE_OFFSET_COUNTER		128*4

#define LPS22HB_PRESSURE_SCALE      40.96f
#define LPS22HB_TEMPERATURE_SCALE   100.0f

void pressure_init(void);

void pressure_test(void);

bool pressure_get_init_status(void);

float pressure_get_pressure_pa(void);

float pressure_get_temperature_c(void);

float pressure_get_height_m(void);

void pressure_read_sensor(void);

void pressure_check_who_am_i_cb(uint8_t *rx);

void pressure_read_sensor_cb(uint8_t *rx);

#endif
