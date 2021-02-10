#ifndef BLDC_H_
#define BLDC_H_

#include "platform.h"
#include <sdk/pid.h>
#include <sdk/frame.h>

typedef enum {
	BLDC_STATE_CALIBRATE_I,
	BLDC_STATE_CALIBRATE_V,
	BLDC_STATE_CALIBRATE_FINISH,
	BLDC_STATE_MEASURE_R,
	BLDC_STATE_MEASURE_L,
	BLDC_STATE_STOP,
	BLDC_STATE_FOC,
	BLDC_STATE_DO_NOTHING
} BldcStateMachine_t;

typedef struct {
	BldcStateMachine_t state;
	void (*state_cb)(void);
} BldcStateDictionaryRow_t;

BldcStateMachine_t bldc_get_active_state(void);

Pid_t *bldc_get_pid(FramePidType_t type);

void bldc_set_pid(FramePidType_t type, float kp, float ki, float kd, float out_limit, float d_filter_coeff);

void bldc_set_i_d(float i_d);

float bldc_get_i_d(void);

float bldc_get_i_q(void);

float bldc_get_speed_rps(void);

float bldc_get_v_ldo_v(void);

float bldc_get_v_vcc_v(void);

float bldc_get_ntc_temperature_c(void);

float bldc_get_up_temperature_c(void);

bool bldc_measure_r_init(void);

bool bldc_measure_l_init(void);

void bldc_set_active_state(BldcStateMachine_t state);

void bldc_set_i_q_ref(float iq);

void bldc_phase_step(uint32_t step);

void bldc_phase_stop(void);

void bldc_adc_irq_hanlder(void);

void bldc_init(void);

bool bldc_status(void);

void bldc_set_enable(bool status);

void bldc_stop_sig(void);

void bldc_start_sig(void);

void bldc_increase_motor_speed_rps(float speed_rps);

#endif
