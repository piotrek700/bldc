#ifndef LPS22HH_H_
#define LPS22HH_H_

#include "platform.h"

#define LPS22HH_OFFSET_COUNTER						128*4

#define LPS22HH_PRESSURE_SCALE      				40.96f
#define LPS22HH_TEMPERATURE_SCALE   				100.0f

#define LPS22HH_SEE_LEVEL_PRESSURE_H				101326.0f
#define LPS22HH_POWER_PRESSURE_PARAMETER_H			0.1902632f
#define LPS22HH_SCALE_PRESSURE_PARAMETER_H			44330.77f

void lps22hh_init(void);

void lps22hh_test(void);

bool lps22hh_get_init_status(void);

float lps22hh_get_pressure_pa(void);

float lps22hh_get_temperature_c(void);

float lps22hh_get_height_m(void);

void lps22hh_read_sensor(void);

void lps22hh_check_who_am_i_cb(uint8_t *rx);

void lps22hh_read_sensor_cb(uint8_t *rx);

#endif
