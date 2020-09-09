#ifndef BLDC_H_
#define BLDC_H_

#include "platform.h"
#include "../Pid/pid.h"
#include "../Frame/frame.h"

#define BLDC_FRAME_SCOPE_BUFF_SIZE		16

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

typedef struct {
	BldcStateMachine state;
	void (*state_cb)(void);
} BldcStateDictionaryRow;

BldcStateMachine bldc_get_active_state(void);

Pid *bldc_get_pid(FramePidType type);

void bldc_set_pid(FramePidType type, float kp, float ki, float kd, float out_limit, float d_filter_coeff);

void bldc_set_i_d(float i_d);

float bldc_get_v_ldo_v(void);

float bldc_get_v_vcc_v(void);

float bldc_get_ntc_temperature_c(void);

float bldc_get_up_temperature_c(void);

bool bldc_measure_r_init(void);

bool bldc_measure_l_init(void);

void bldc_set_active_state(BldcStateMachine state);

void bldc_set_i_q_ref(float iq);

void bldc_phase_step(uint32_t step);

void bldc_phase_stop(void);

void bldc_adc_irq_hanlder(void);

void bldc_init(void);

bool bldc_status(void);

void bldc_set_enable(bool status);

FrameDisplayChannelsData4 * bldc_get_scope_4ch_frame(uint32_t index);

bool bldc_get_frame_ready(uint32_t index);

void bldc_get_frame_ready_clear(uint32_t index);

void bldc_stop_sig(void);

void bldc_start_sig(void);

void bldc_increase_motor_speed_rps(float speed_rps);

void bldc_scope_send_data(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4);

#endif
