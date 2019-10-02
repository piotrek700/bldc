#include "pressure.h"
#include "pressure_tran.h"

static bool init_status = false;
static volatile float pressure = 0;
static volatile float temperature = 0;
static volatile float height = 0;

//1
void pressure_check_who_am_i_cb(uint8_t *rx) {
	if (rx[1] != LPS22HB_WHO_AM_I_RESPONSE) {
		debug_error(PRESSURE_SENOR_NOT_DETECTED);
	}
}

//3
void pressure_read_sensor_cb(uint8_t *rx) {
	int32_t *rx_pressure;

	rx_pressure = (int32_t *) rx;
	(*rx_pressure) = (*rx_pressure) >> 8;
	pressure = (float) (*rx_pressure) / LPS22HB_PRESSURE_SCALE;

	int16_t *rx_temperature;
	rx_temperature = (int16_t *) (rx + 4);
	temperature = (float) (*rx_temperature) / LPS22HB_TEMPERATURE_SCALE;

	height = 44330.77f * (1.0f - powf(pressure / 101326.0f, 0.1902632f));	//TODO define all parameters
}

static void pressure_check_who_am_i(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_who_am_i);
}

static void pressure_lps22hb_init(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_ctrl1_init);
}

void pressure_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	pressure_check_who_am_i();
	pressure_lps22hb_init();

	pressure_test();

	init_status = true;
}

float pressure_get_pressure_pa(void) {
	return pressure;
}

float pressure_get_temperature_c(void) {
	return temperature;
}

float pressure_get_height_m(void) {
	return height;
}

void pressure_read_sensor(void) {
	spi_add_transaction((SpiTransactionRecord *) &record_read_sensor);
}

void pressure_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool pressure_get_init_status(void) {
	return init_status;
}
