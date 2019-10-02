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
	BLDC_STATE_INJECT_ROTOR_POSITION,
	BLDC_STATE_INJECT_ROTATION_LOOP,
	BLDC_STATE_SILENCE_BEFORE_INJECT,
	BLDC_STATE_INIT_ROTOR_POSITION,
	BLDC_STATE_BEMF_ROTATION_LOOP,
	BLDC_STATE_FOC,
	BLDC_STATE_STOP,
	BLDC_STATE_MEASURE_R,
	BLDC_STATE_MEASURE_L,
	BLDC_STATE_DO_NOTHING
} BldcStateMachine;

typedef enum{
	BLDC_INJECT_IAP=0,
	BLDC_INJECT_IBP,
	BLDC_INJECT_ICP,
	BLDC_INJECT_IAN,
	BLDC_INJECT_IBN,
	BLDC_INJECT_ICN,
	BLDC_INJECT_NONE
}BldcInjectPhase;

void bldc_task(void);

void bldc_task_v2(void);

void bldc_set_injected_phase(BldcInjectPhase phase);

void bldc_phase_step(uint32_t step);

void bldc_phase_stop(void);

#endif
