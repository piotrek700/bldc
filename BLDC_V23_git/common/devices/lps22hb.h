#ifndef LPS22HB_H_
#define LPS22HB_H_

#include "platform.h"

#define LPS22HB_OFFSET_COUNTER						128*4

#define LPS22HB_PRESSURE_SCALE      				40.96f
#define LPS22HB_TEMPERATURE_SCALE   				100.0f

#define LPS22HB_SEE_LEVEL_PRESSURE_H				101326.0f
#define LPS22HB_POWER_PRESSURE_PARAMETER_H			0.1902632f
#define LPS22HB_SCALE_PRESSURE_PARAMETER_H			44330.77f

void lps22hb_init(void);

void lps22hb_test(void);

bool lps22hb_get_init_status(void);

float lps22hb_get_pressure_pa(void);

float lps22hb_get_temperature_c(void);

float lps22hb_get_height_m(void);

void lps22hb_read_sensor(void);

void lps22hb_check_who_am_i_cb(uint8_t *rx);

void lps22hb_read_sensor_cb(uint8_t *rx);

#endif
