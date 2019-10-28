#ifndef BLDC_H_
#define BLDC_H_

#include "stm32f30x.h"
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../Rybos/rybos.h"
#include "../Drv8301/drv8301.h"
#include "../Adc/adc.h"
#include <stdbool.h>
#include "math.h"
#include "arm_math.h"
#include "../Frame/frame.h"

typedef enum {
	BLDC_STATE_CALIBRATE_I,
	BLDC_STATE_CALIBRATE_V,
	BLDC_STATE_CALIBRATE_FINISH,
	BLDC_STATE_MEASURE_R,
	BLDC_STATE_MEASURE_L,
	BLDC_STATE_STOP,
	BLDC_STATE_FOC,
	BLDC_STATE_DO_NOTHING
} BldcStateMachine;

typedef struct{
	BldcStateMachine state;
	void (*state_cb)(void);
}BldcStateDictionaryRow;

float bldc_get_v_ldo_v(void);

float bldc_get_v_vcc_v(void);

float bldc_get_ntc_temperature_c(void);

float bldc_get_up_temperature_c(void);

bool bldc_measure_r_init(void) ;

bool bldc_measure_l_init(void);

void bldc_set_active_state(BldcStateMachine state);






void bldc_phase_step(uint32_t step);

void bldc_phase_stop(void);

void bldc_adc_irq_hanlder(void);



void bldc_init(void);

bool bldc_status(void);

void bldc_set_enable(bool status);

#endif
