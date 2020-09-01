#include <Pressure/lps22hb.h>
#include <Pressure/lps22hb_tran.h>
#include "../Debug/debug.h"
#include "../Spi/spi.h"
#include "math.h"

static bool init_status = false;
static volatile float pressure = 0;
static volatile float temperature = 0;
static volatile float height = 0;

//1
void lps22hb_check_who_am_i_cb(uint8_t *rx) {
	if (rx[1] != LPS22HB_WHO_AM_I_RESPONSE) {
		debug_error(PRESSURE_SENOR_NOT_DETECTED);
	}
}

//3
void lps22hb_read_sensor_cb(uint8_t *rx) {
	int32_t *rx_pressure;

	rx_pressure = (int32_t *) rx;
	(*rx_pressure) = (*rx_pressure) >> 8;
	pressure = (float) (*rx_pressure) / LPS22HB_PRESSURE_SCALE;

	int16_t *rx_temperature;
	rx_temperature = (int16_t *) (rx + 4);
	temperature = (float) (*rx_temperature) / LPS22HB_TEMPERATURE_SCALE;

	height = LPS22HB_SCALE_PRESSURE_PARAMETER_H * (1.0f - powf(pressure / LPS22HB_SEE_LEVEL_PRESSURE_H, LPS22HB_POWER_PRESSURE_PARAMETER_H));
}

static void lps22hb_check_who_am_i(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_who_am_i);
}

static void lps22hb_lps22hb_init(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl1_init);
}

void lps22hb_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	lps22hb_check_who_am_i();
	lps22hb_lps22hb_init();

	lps22hb_test();

	init_status = true;
}

float lps22hb_get_pressure_pa(void) {
	return pressure;
}

float lps22hb_get_temperature_c(void) {
	return temperature;
}

float lps22hb_get_height_m(void) {
	return height;
}

void lps22hb_read_sensor(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_read_sensor);
}

void lps22hb_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool lps22hb_get_init_status(void) {
	return init_status;
}
