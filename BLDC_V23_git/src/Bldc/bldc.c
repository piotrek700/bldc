#include "bldc.h"
#include "utils.h"
#include "math.h"
#include "arm_math.h"
#include <stdio.h>
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../Rybos/rybos.h"
#include "../Drv8301/drv8301.h"
#include "../Adc/adc.h"

//Constant
#define ONE_BY_SQRT3							0.57735026919f
#define TWO_BY_SQRT3							1.15470053838f
#define ONE_OVER_3							    0.66666666667f
#define SQRT3_BY_2								0.86602540378f
#define SQRT3_BY_2_MUL_2_OVER_3					0.57735026919f

//Calibration
#define BEMF_V_CALIBRATION_WAIT_TIME_MS			10
#define BEMF_I_CALIBRATION_DELAY_TIME_MS		100

#define BLDC_R_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_R_SAMPLES					1024*4
#define BLDC_MEASURE_R_THETA_DEG				330.0f
#define BLDC_MEASURE_R_CURRENT_A				20.0f

#define BLDC_L_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_L_WAIT_CYCLES				64
#define BLDC_MEASURE_L_DUTY						0.4f
#define BLDC_MEASURE_L_SAMPLES					1024*BLDC_MEASURE_L_WAIT_CYCLES
#define BLDC_MAX_DUTY							0.8f									//Can be increased
#define BLDC_MAX_DQ_STEP  						0.01f

//Silver-blue
//#define MOTOR_R								(0.093f*0.85f)							//Single arm value, compensation 0.85 - why? - i dont know
//#define MOTOR_L								9.86e-6f								//Single arm value
//#define MOTOR_LAMBDA							0.0006f
//#define MOTOR_PID_TIME_CONSTANT				0.001f

//MT2266II MAX
#define MOTOR_R									0.0622f
#define MOTOR_L									12.30e-6f
#define MOTOR_LAMBDA							0.0009f
#define MOTOR_PID_TIME_CONSTANT					0.001f

//FOC
#define BLDC_DT									(1.0f/(float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)
#define BLDC_VDQ_MAX_LIMIT						ONE_BY_SQRT3 							//(SQRT3_BY_2 * 2.0f / 3.0f)

#define MOTOR_KA								(MOTOR_L/MOTOR_PID_TIME_CONSTANT)
#define MOTOR_KB								(MOTOR_R/MOTOR_L)

#define BLDC_PID_KP 							MOTOR_KA
#define BLDC_PID_KI								MOTOR_KB * MOTOR_KA

//Speed PID
#define BLDC_SPEED_PID_KP						1.8f	// 2.5
#define BLDC_SPEED_PID_KI						0.0f	//10.0
#define BLDC_SPEED_PID_KD						0.09f	//0.15
#define BLDC_SPEED_PID_I_LIMIT					10.0f

#define BLDC_PID_I_LIMIT						BLDC_VDQ_MAX_LIMIT
#define BLDC_PID_OUT_LIMIT 						BLDC_VDQ_MAX_LIMIT

//LPF
#define BLDC_DQ_LPF_CUTOFF_FREQ					500.0f
#define BLDC_VCC_LPF_CUTOFF_FREQ				100.0f

#define BLDC_DQ_LPF_ALPHA						(BLDC_DQ_LPF_CUTOFF_FREQ/(BLDC_DQ_LPF_CUTOFF_FREQ+((float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)/(2.0f*(float)M_PI)))
#define BLDC_LDO_VCC_LPF_ALPHA					(BLDC_DQ_LPF_CUTOFF_FREQ/(BLDC_DQ_LPF_CUTOFF_FREQ+((float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)/(2.0f*(float)M_PI)))

#define BLDC_IQ_MAX_FOR_RUN						20.0f

#define BLDC_STARTUP_TIME_MS					200
#define BLDC_START_TIEMOUT_MS 					1000
#define BLDC_STARTUP_SPEED_RPS					5.0f

#define MOTOR_NUMER_OF_POLES					7.0f
#define MOTOR_MIN_SPEED_TARGET_RPS				10.0f

//PLL
#define BLDC_PLL_KP								2000.0f
#define BLDC_PLL_KI								30000.0f

//Observer
#define BLDC_OBSERVER_GAMMA  					(1.0f/(MOTOR_PID_TIME_CONSTANT * MOTOR_LAMBDA * MOTOR_LAMBDA))

//Variables
static float v1_bemf_h = 0;
static float v2_bemf_h = 0;
static float v3_bemf_h = 0;
static float v_bemf_h_ldo = 0;
static float v_bemf_h_vcc = 0;

static float v1_bemf_offset = 0;
static float v2_bemf_offset = 0;
static float v3_bemf_offset = 0;
static float v_bemf_offset_ldo = 0;
static float v_bemf_offset_vcc = 0;

//CCM variables
CCMRAM_VARIABLE static float p1_i_offset = 0;
CCMRAM_VARIABLE static float p3_i_offset = 0;
CCMRAM_VARIABLE static float i_offset_ldo = 0;

CCMRAM_VARIABLE static float i_q_ref_rc = 0.0;
CCMRAM_VARIABLE static float tetha_deg = 0;

CCMRAM_VARIABLE static float i_d_ref = 0;
CCMRAM_VARIABLE static float i_q_ref = 0;
CCMRAM_VARIABLE static float i_q_max = 0;
CCMRAM_VARIABLE static float i_d_err_acc = 0;			//TODO remove
CCMRAM_VARIABLE static float i_q_err_acc = 0;			//TODO remove
CCMRAM_VARIABLE static float i_d_lpf = 0;				//TODO remove
CCMRAM_VARIABLE static float i_q_lpf = 0;				//TODO remove

CCMRAM_VARIABLE static float adc_vref_mul_vrefint_cal = 0;
CCMRAM_VARIABLE static float one_over_adc_temp_call = 0;

//Cannot be zero at start
CCMRAM_VARIABLE static float i_alpha = 0.01f;
//Cannot be zero at start
CCMRAM_VARIABLE static float i_beta = -0.01f;
//Cannot be zero at start
CCMRAM_VARIABLE static float v_alpha = 0.2f;
//Cannot be zero at start
CCMRAM_VARIABLE static float v_beta = -0.2f;

CCMRAM_VARIABLE static float mod_alpha = 0;
CCMRAM_VARIABLE static float mod_beta = 0;

//ADC reads
CCMRAM_VARIABLE static float v_vcc_v = 0;
CCMRAM_VARIABLE static float v_ldo_v = 0;
CCMRAM_VARIABLE static float up_temperature_c = 0;
CCMRAM_VARIABLE static float ntc_temperature_c = 0;

//Observer
CCMRAM_VARIABLE static float x1 = 0;
CCMRAM_VARIABLE static float x2 = 0;

//PLL
CCMRAM_VARIABLE static float pll_phase_rad = 0;
CCMRAM_VARIABLE static float pll_speed_rad = 0;

//FOC
CCMRAM_VARIABLE static uint32_t foc_start_time = 0;
CCMRAM_VARIABLE static float speed_direction = -1.0f;
CCMRAM_VARIABLE static float motor_speed_target_rps = 0.0f;
CCMRAM_VARIABLE static float theta_over_deg = 0;
CCMRAM_VARIABLE static float motor_speed_rps = 0;
CCMRAM_VARIABLE static float magnetic_speed_rps = 0;

//Measurements
static uint32_t meaure_r_start_time = 0;
static uint32_t measure_r_cnt = 0;
static float measure_r_current_avr = 0;
static float measure_r_voltage_avr = 0;

static uint32_t meaure_l_start_time = 0;
static uint32_t measure_l_cnt = 0;
static float measure_l_i1_avr_1 = 0;
static float measure_l_i3_avr_1 = 0;
static float measure_l_vcc_avr_1 = 0;

static float measure_l_i1_avr_2 = 0;
static float measure_l_i3_avr_2 = 0;
static float measure_l_vcc_avr_2 = 0;

//Scope
static volatile FrameDisplayChannelsData4 scope_frame_buff[BLDC_FRAME_SCOPE_BUFF_SIZE];
static volatile bool scope_frame_ready_buff[BLDC_FRAME_SCOPE_BUFF_SIZE];
static volatile uint32_t scope_frame_buff_depth = 0;
static volatile uint32_t scope_frame_buff_max_depth = 0;

//FLux
static float linkage_bemf_a = 0.0;
static float linkage_bemf_b = 0.0;
static float linkage_bemf_c = 0.0;

static float linkage = 0;

//PID
CCMRAM_VARIABLE PID_STATIC_INIT(pid_speed, BLDC_SPEED_PID_KP, BLDC_SPEED_PID_KI, BLDC_SPEED_PID_KD, BLDC_SPEED_PID_I_LIMIT, 0.0f);
CCMRAM_VARIABLE PID_STATIC_INIT(pid_d, BLDC_PID_KP, BLDC_PID_KI, 0.0f, BLDC_PID_I_LIMIT * BLDC_MAX_DUTY, 0);
CCMRAM_VARIABLE PID_STATIC_INIT(pid_q, BLDC_PID_KP, BLDC_PID_KI, 0.0f, BLDC_PID_I_LIMIT * BLDC_MAX_DUTY, 0);

//States declaration
static void bldc_state_calibrate_i(void);
static void bldc_state_calibrate_v(void);
static void bldc_state_calibrate_finish(void);
static void bldc_state_measure_r(void);
static void bldc_state_measure_l(void);
static void bldc_state_stop(void);
static void bldc_state_foc(void);
static void bldc_state_do_nothing(void);

static void bldc_flux_linkage_measurement(float v_d, float v_q, float i_d, float i_q);

CCMRAM_VARIABLE static BldcStateDictionaryRow state_dictionary[] = {
		{ BLDC_STATE_CALIBRATE_I, bldc_state_calibrate_i },
		{ BLDC_STATE_CALIBRATE_V, bldc_state_calibrate_v },
		{ BLDC_STATE_CALIBRATE_FINISH, bldc_state_calibrate_finish },
		{ BLDC_STATE_MEASURE_R, bldc_state_measure_r },
		{ BLDC_STATE_MEASURE_L, bldc_state_measure_l },
		{ BLDC_STATE_STOP, bldc_state_stop },
		{ BLDC_STATE_FOC, bldc_state_foc },
		{ BLDC_STATE_DO_NOTHING, bldc_state_do_nothing }
};

//BLDC state
CCMRAM_VARIABLE static void (*bldc_active_state_cb)(void) = bldc_state_do_nothing;
CCMRAM_VARIABLE static BldcStateMachine bldc_active_state = BLDC_STATE_DO_NOTHING;

BldcStateMachine bldc_get_active_state(void){
	return bldc_active_state;
}

Pid *bldc_get_pid(FramePidType type) {
	if (type == FRAME_PID_TYPE_BLDC_SPEED) {
		return &pid_speed;
	} else if (type == FRAME_PID_TYPE_BLDC_DQ) {
		return &pid_d;
	}

	return 0;
}

void bldc_set_pid(FramePidType type, float kp, float ki, float kd, float out_limit, float d_filter_coeff) {
	if (type == FRAME_PID_TYPE_BLDC_SPEED) {
		pid_set_param(&pid_speed, kp, ki, kd, out_limit, d_filter_coeff);

	} else if (type == FRAME_PID_TYPE_BLDC_DQ) {
		pid_set_param(&pid_d, kp, ki, kd, out_limit, d_filter_coeff);
		pid_set_param(&pid_q, kp, ki, kd, out_limit, d_filter_coeff);
	}
}

void bldc_set_i_d(float i_d) {
	i_q_ref = i_d;
}

float bldc_get_v_ldo_v(void) {
	return v_ldo_v;
}

float bldc_get_v_vcc_v(void) {
	return v_vcc_v;
}

float bldc_get_ntc_temperature_c(void) {
	return ntc_temperature_c;
}

float bldc_get_up_temperature_c(void) {
	return up_temperature_c;
}

CCMRAM_FUCNTION static void bldc_svm(float alpha, float beta, uint32_t half_period, uint32_t* t_a_out, uint32_t* t_b_out, uint32_t* t_c_out,
		uint32_t *svm_sector) {
	uint32_t sector;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant 1
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 2;
			} else {
				sector = 1;
			}
		} else {
			//quadrant 2
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 3;
			} else {
				sector = 2;
			}
		}
	} else {
		if (alpha >= 0.0f) {
			//quadrant 5
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 5;
			} else {
				sector = 6;
			}
		} else {
			//quadrant 3
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 4;
			} else {
				sector = 5;
			}
		}
	}

	//PWM timings
	uint32_t t_a, t_b, t_c;

	switch (sector) {
	case 1: {
		//sector 1-2
		uint32_t t1 = (uint32_t) ((alpha - ONE_BY_SQRT3 * beta) * half_period);
		uint32_t t2 = (uint32_t) ((TWO_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_a = (half_period - t1 - t2) / 2;
		t_b = t_a + t1;
		t_c = t_b + t2;
		break;
	}

	case 2: {
		//sector 2-3
		uint32_t t2 = (uint32_t) ((alpha + ONE_BY_SQRT3 * beta) * half_period);
		uint32_t t3 = (uint32_t) ((-alpha + ONE_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_b = (half_period - t2 - t3) / 2;
		t_a = t_b + t3;
		t_c = t_a + t2;
		break;
	}

	case 3: {
		//sector 3-4
		uint32_t t3 = (uint32_t) ((TWO_BY_SQRT3 * beta) * half_period);
		uint32_t t4 = (uint32_t) ((-alpha - ONE_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_b = (half_period - t3 - t4) / 2;
		t_c = t_b + t3;
		t_a = t_c + t4;
		break;
	}

	case 4: {
		//sector 4-5
		uint32_t t4 = (uint32_t) ((-alpha + ONE_BY_SQRT3 * beta) * half_period);
		uint32_t t5 = (uint32_t) ((-TWO_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_c = (half_period - t4 - t5) / 2;
		t_b = t_c + t5;
		t_a = t_b + t4;
		break;
	}

	case 5: {
		//sector 5-6
		uint32_t t5 = (uint32_t) ((-alpha - ONE_BY_SQRT3 * beta) * half_period);
		uint32_t t6 = (uint32_t) ((alpha - ONE_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_c = (half_period - t5 - t6) / 2;
		t_a = t_c + t5;
		t_b = t_a + t6;
		break;
	}

	case 6: {
		//sector 6-1
		uint32_t t6 = (uint32_t) ((-TWO_BY_SQRT3 * beta) * half_period);
		uint32_t t1 = (uint32_t) ((alpha + ONE_BY_SQRT3 * beta) * half_period);

		//PWM timings
		t_a = (half_period - t6 - t1) / 2;
		t_c = t_a + t1;
		t_b = t_c + t6;
		break;
	}

	default:
		debug_error(BLDC_SECTOR_NOT_SUPPORTED);
	}

	*t_a_out = t_a;
	*t_b_out = t_b;
	*t_c_out = t_c;
	*svm_sector = sector;
}

static void bldc_enable_all_pwm_output(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
}

static void bldc_disable_all_pwm_output(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}

static void bldc_stop_pwm(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}

static void bldc_state_calibrate_v(void) {
	static uint32_t time = 0;
	static uint32_t p_offset_cnt = 0;
	static bool h_complete = false;

	if (time == 0) {
		time = tick_get_time_ms();
		p_offset_cnt = 0;

		if (h_complete) {
			//Measure L
			drv8301_set_pwm(0, 0, 0);

		} else {
			//Measure H
			drv8301_set_pwm(DRV8301_PWM_3F_PWM_MAX, DRV8301_PWM_3F_PWM_MAX, DRV8301_PWM_3F_PWM_MAX);
		}

		bldc_enable_all_pwm_output();
		DRV8301_PWM_UPDATE_EVENT;
	} else if (tick_get_time_ms() - time > BEMF_V_CALIBRATION_WAIT_TIME_MS) {
		if (h_complete) {
			if (p_offset_cnt < ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_offset += ADC_INJ_P1_BEMF;
				v2_bemf_offset += ADC_INJ_P2_BEMF;
				v3_bemf_offset += ADC_INJ_P3_BEMF;
				v_bemf_offset_ldo += v_ldo_v;
				v_bemf_offset_vcc += v_vcc_v;

				p_offset_cnt++;
			} else if (p_offset_cnt == ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_offset /= (float) p_offset_cnt;
				v2_bemf_offset /= (float) p_offset_cnt;
				v3_bemf_offset /= (float) p_offset_cnt;
				v_bemf_offset_ldo /= (float) p_offset_cnt;
				v_bemf_offset_vcc /= (float) p_offset_cnt;

				float v1 = v1_bemf_offset * ADC_V_GAIN;
				v1 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				float v2 = v2_bemf_offset * ADC_V_GAIN;
				v2 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				float v3 = v3_bemf_offset * ADC_V_GAIN;
				v3 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				//GND floor measurement
				printf("Calibration BEMF L V1[V]:   %.6f\n", (double) v1);
				printf("Calibration BEMF L V2[V]:   %.6f\n", (double) v2);
				printf("Calibration BEMF L V3[V]:   %.6f\n", (double) v3);
				printf("Calibration BEMF L VLDO[V]: %.3f\n", (double) v_bemf_offset_ldo);
				printf("Calibration BEMF L VCC[V]:  %.3f\n", (double) v_bemf_offset_vcc);

				bldc_set_active_state(BLDC_STATE_CALIBRATE_FINISH);
			}
		} else {
			if (p_offset_cnt < ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_h += ADC_INJ_P1_BEMF;
				v2_bemf_h += ADC_INJ_P2_BEMF;
				v3_bemf_h += ADC_INJ_P3_BEMF;
				v_bemf_h_ldo += v_ldo_v;
				v_bemf_h_vcc += v_vcc_v;

				p_offset_cnt++;
			} else if (p_offset_cnt == ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_h /= (float) p_offset_cnt;
				v2_bemf_h /= (float) p_offset_cnt;
				v3_bemf_h /= (float) p_offset_cnt;
				v_bemf_h_ldo /= (float) p_offset_cnt;
				v_bemf_h_vcc /= (float) p_offset_cnt;

				float v1 = v1_bemf_h * ADC_V_GAIN;
				v1 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				float v2 = v2_bemf_h * ADC_V_GAIN;
				v2 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				float v3 = v3_bemf_h * ADC_V_GAIN;
				v3 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				//VCC measurement
				printf("Calibration BEMF H V1[V]:   %.3f\n", (double) v1);
				printf("Calibration BEMF H V2[V]:   %.3f\n", (double) v2);
				printf("Calibration BEMF H V3[V]:   %.3f\n", (double) v3);
				printf("Calibration BEMF H VLDO[V]: %.3f\n", (double) v_bemf_h_ldo);
				printf("Calibration BEMF H VCC[V]:  %.3f\n", (double) v_bemf_h_vcc);

				time = 0;
				h_complete = true;
			}
		}
	}
}

static void bldc_state_calibrate_i(void) {
	//Delay
	static uint32_t timer = 0;
	if (timer == 0) {
		timer = tick_get_time_ms();
		return;
	}

	//Wait for output stabilization
	if (tick_get_time_ms() - timer < BEMF_I_CALIBRATION_DELAY_TIME_MS) {
		return;
	}

	//Calibrate
	static uint32_t p_offset_cnt = 0;

	if (p_offset_cnt < ADC_I_OFFSET_COUNTER_MAX) {
		p1_i_offset += ADC_INJ_P1_I;
		p3_i_offset += ADC_INJ_P3_I;
		i_offset_ldo += v_ldo_v;
		p_offset_cnt++;
	} else if (p_offset_cnt == ADC_I_OFFSET_COUNTER_MAX) {
		p1_i_offset /= (float) p_offset_cnt;
		p3_i_offset /= (float) p_offset_cnt;
		i_offset_ldo /= (float) p_offset_cnt;

		float p1 = ((float) p1_i_offset - ADC_MAX_VALUE / 2) / ADC_I_GAIN;
		p1 *= i_offset_ldo / ADC_MAX_VALUE / ADC_I_R_OHM;

		float p3 = ((float) p3_i_offset - ADC_MAX_VALUE / 2) / ADC_I_GAIN;
		p3 *= i_offset_ldo / ADC_MAX_VALUE / ADC_I_R_OHM;

		printf("Calibration I1[A]:     %.3f\n", (double) p1);
		printf("Calibration I3[A]:     %.3f\n", (double) p3);
		printf("Calibration I VLDO[V]: %.3f\n", (double) i_offset_ldo);

		drv8301_i_calibration_disable();
		bldc_set_active_state(BLDC_STATE_CALIBRATE_V);
	}
}

static void bldc_state_stop(void) {
	drv8301_set_pwm(0, 0, 0);

	bldc_stop_pwm();
	DRV8301_PWM_UPDATE_EVENT;

	bldc_set_active_state(BLDC_STATE_DO_NOTHING);
}

bool bldc_measure_r_init(void) {
	if (bldc_active_state != BLDC_STATE_DO_NOTHING) {
		return false;
	}

	//Set flow VDC->L->L->I1, VDC->L->L->I2, not in reverse to avoid V diode drop
	tetha_deg = BLDC_MEASURE_R_THETA_DEG;
	i_d_ref = 0;
	i_q_ref = BLDC_MEASURE_R_CURRENT_A;

	i_d_lpf = 0;
	i_q_lpf = 0;

	i_d_err_acc = 0;
	i_q_err_acc = 0;

	measure_r_current_avr = 0;
	measure_r_voltage_avr = 0;

	measure_r_cnt = 0;
	meaure_r_start_time = tick_get_time_ms();

	bldc_enable_all_pwm_output();
	DRV8301_PWM_UPDATE_EVENT;

	bldc_set_active_state(BLDC_STATE_MEASURE_R);
	return true;
}

//TODO here is a problem with  accuracy. The result is higher and need compensation *0.86
//The results stabilize with higher current because the coverage of the duty is higher i think so
static void bldc_state_measure_r(void) {
	//Current limit
	if (i_q_ref > i_q_max) {
		i_q_ref = i_q_max;
	} else if (i_q_ref < -i_q_max) {
		i_q_ref = -i_q_max;
	}

	//Check saturation
	float i_dq_mag = 1e-10f;
	arm_sqrt_f32(i_d_ref * i_d_ref + i_q_ref * i_q_ref, &i_dq_mag);

	if (i_dq_mag < 1e-10f) {
		i_dq_mag = 1e-10f;
	}

	if (i_dq_mag > i_q_max) {
		//Risk of division by zero v_dq_mag
		float dq_scale = i_q_max / i_dq_mag;
		i_d_ref *= dq_scale;
		i_q_ref *= dq_scale;
	}

	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) * v_ldo_v / ( ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);
	//float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) * v_ldo_v / (  ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) * v_ldo_v / ( ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);

	//static float p1_i_lpf = 0;
	//p1_i_lpf = p1_i_lpf * 0.4f + 0.6f * p1_i;
	//p1_i = p1_i_lpf;

	//static float p3_i_lpf = 0;
	//p3_i_lpf = p3_i_lpf * 0.4f + 0.6f * p3_i;
	//p3_i = p3_i_lpf;

	//Clarke Transform
	i_alpha = p1_i;
	i_beta = ONE_BY_SQRT3 * p1_i + TWO_BY_SQRT3 * p3_i;

	//Park Transform
	float sin_tetha;
	float cos_tetha;
	arm_sin_cos_f32(tetha_deg, &sin_tetha, &cos_tetha);

	float i_d = i_alpha * cos_tetha + i_beta * sin_tetha;
	float i_q = i_beta * cos_tetha - i_alpha * sin_tetha;

	//Current LPF
	float lpf_val = 1.0f - BLDC_DQ_LPF_ALPHA;
	i_d_lpf = BLDC_DQ_LPF_ALPHA * i_d + lpf_val * i_d_lpf;
	i_q_lpf = BLDC_DQ_LPF_ALPHA * i_q + lpf_val * i_q_lpf;

	//This reduce noise during standstill
	i_d = i_d_lpf;
	i_q = i_q_lpf;

	//PID P error
	float i_d_err = i_d_ref - i_d;
	float i_q_err = i_q_ref - i_q;

	float v_d = i_d_err * BLDC_PID_KP + i_d_err_acc;
	float v_q = i_q_err * BLDC_PID_KP + i_q_err_acc;

	//PID I error
	i_d_err_acc += i_d_err * BLDC_DT * BLDC_PID_KI;
	i_q_err_acc += i_q_err * BLDC_DT * BLDC_PID_KI;

	//Maximum limitation
	float v_dq_mag = 1e-10f;
	arm_sqrt_f32(v_d * v_d + v_q * v_q, &v_dq_mag);
	float v_dq_max = BLDC_VDQ_MAX_LIMIT * v_vcc_v * BLDC_MAX_DUTY;

	if (v_dq_mag < 1e-10f) {
		v_dq_mag = 1e-10f;
	}

	if (v_dq_mag > v_dq_max) {
		//Risk of division by zero v_dq_mag
		float dq_scale = v_dq_max / v_dq_mag;
		v_d *= dq_scale;
		v_q *= dq_scale;
	}

	float v_mod = 1.0f / ((2.0f / 3.0f) * v_vcc_v);
	float mod_d = v_d * v_mod;
	float mod_q = v_q * v_mod;

	//PID I limit
	float i_lim_mul_vcc = BLDC_PID_I_LIMIT * v_vcc_v * BLDC_MAX_DUTY;
	if (i_d_err_acc > i_lim_mul_vcc) {
		i_d_err_acc = i_lim_mul_vcc;
	} else if (i_d_err_acc < -i_lim_mul_vcc) {
		i_d_err_acc = -i_lim_mul_vcc;
	}

	if (i_q_err_acc > i_lim_mul_vcc) {
		i_q_err_acc = i_lim_mul_vcc;
	} else if (i_q_err_acc < -i_lim_mul_vcc) {
		i_q_err_acc = -i_lim_mul_vcc;
	}

	//Dead time compensation
	mod_alpha = cos_tetha * mod_d - sin_tetha * mod_q;
	mod_beta = cos_tetha * mod_q + sin_tetha * mod_d;

	float i_alpha_filter = cos_tetha * i_d_ref - sin_tetha * i_q_ref;
	float i_beta_filter = cos_tetha * i_q_ref + sin_tetha * i_d_ref;

	float ia_filter = i_alpha_filter;
	float ib_filter = -0.5f * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
	float ic_filter = -0.5f * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

	float mod_alpha_filter_sgn = (2.0f / 3.0f) * SIGN(ia_filter) - (1.0f / 3.0f) * SIGN(ib_filter) - (1.0f / 3.0f) * SIGN(ic_filter);
	float mod_beta_filter_sgn = ONE_BY_SQRT3 * SIGN(ib_filter) - ONE_BY_SQRT3 * SIGN(ic_filter);
	//25 -  0.17361111e-6f	-checked by oscilloscope
	//150 - 1.1944e-6f   	-checked by oscilloscope add 80nS from DRV
	//TODO add macro to calculate this value
	float mod_comp_fact = 0.17361111e-6f * (float) DRV8301_PWM_3F_SWITCHING_FREQ_HZ;
	float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
	float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

	//Correct dead time
	mod_alpha += mod_alpha_comp;
	mod_beta += mod_beta_comp;

	//float vd_10 = mod_alpha * cos_tetha + mod_beta*sin_tetha;
	//float vq_10 = mod_beta * cos_tetha - mod_alpha*sin_tetha;

	//SVM
	uint32_t duty1, duty2, duty3, svm_sector;
	bldc_svm(mod_alpha, mod_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);

	//Set PWM
	TIM1->CR1 |= TIM_CR1_UDIS;
	drv8301_set_pwm(duty1, duty2, duty3);
	TIM1->CR1 &= ~TIM_CR1_UDIS;

	if (tick_get_time_ms() - meaure_r_start_time > BLDC_R_MEASUREMENT_START_TIME_MS) {
		float i_dc_link = 1e-10f;
		arm_sqrt_f32(i_d * i_d + i_q * i_q, &i_dc_link);

		float v_dc_link = 1e-10f;
		arm_sqrt_f32(v_d * v_d + v_q * v_q, &v_dc_link);

		//v_dc_link -= mod_comp_fact * v_vcc_v * 2.0f / 3.0f;

		static float p1_i_avr = 0;
		static float p3_i_avr = 0;
		static float p2_v_avr = 0;

		measure_r_current_avr += i_dc_link;
		measure_r_voltage_avr += v_dc_link;

		p1_i_avr += p1_i;
		p3_i_avr += p3_i;

		//Negative dead-time compensation because when dt is present, controller driver higher duty to reach the current so we need to compensate duty cycle
		float duty_avr = duty1 + duty2 + duty3;
		p2_v_avr += (((float) duty2 - duty_avr / 3.0f) / (float) DRV8301_PWM_3F_PWM_MAX * v_vcc_v) - mod_comp_fact * v_vcc_v;
		measure_r_cnt++;

		if (measure_r_cnt == BLDC_MEASURE_R_SAMPLES) {
			printf("Measured R AVR Vdq[V]:  %.6f\n", (double) (measure_r_voltage_avr / (float) measure_r_cnt));
			printf("Measured R AVR Idq[A]:  %.6f\n", (double) (measure_r_current_avr / (float) measure_r_cnt));
			printf("Measured R AVR R1[Ohm]: %.6f\n", (double) (measure_r_voltage_avr / measure_r_current_avr));
			printf("Measured R AVR R2[Ohm]: %.6f\n", (double) (p2_v_avr / (p1_i_avr + p3_i_avr)));

			bldc_set_active_state(BLDC_STATE_STOP);
		}
	}
}

bool bldc_measure_l_init(void) {
	if (bldc_active_state != BLDC_STATE_DO_NOTHING) {
		return false;
	}

	measure_l_i1_avr_1 = 0;
	measure_l_i3_avr_1 = 0;
	measure_l_vcc_avr_1 = 0;

	measure_l_i1_avr_2 = 0;
	measure_l_i3_avr_2 = 0;
	measure_l_vcc_avr_2 = 0;

	measure_l_cnt = 0;
	meaure_l_start_time = tick_get_time_ms();

	bldc_enable_all_pwm_output();
	DRV8301_PWM_UPDATE_EVENT;

	bldc_set_active_state(BLDC_STATE_MEASURE_L);
	return true;
}

static void bldc_state_measure_l(void) {
	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//I3
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	if (tick_get_time_ms() - meaure_l_start_time > BLDC_L_MEASUREMENT_START_TIME_MS) {

		if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 0) {
			drv8301_set_pwm(BLDC_MEASURE_L_DUTY * DRV8301_PWM_3F_PWM_MAX, 0, BLDC_MEASURE_L_DUTY * DRV8301_PWM_3F_PWM_MAX);
			DRV8301_PWM_UPDATE_EVENT;
		} else if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 2) {
			measure_l_i1_avr_1 += -p1_i;
			measure_l_i3_avr_1 += -p3_i;
			measure_l_vcc_avr_1 += v_vcc_v;

			drv8301_set_pwm(0, 0, 0);
			DRV8301_PWM_UPDATE_EVENT;
		} else if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 3) {
			measure_l_i1_avr_2 += -p1_i;
			measure_l_i3_avr_2 += -p3_i;
			measure_l_vcc_avr_2 += v_vcc_v;
		}

		measure_l_cnt++;

		if (measure_l_cnt == BLDC_MEASURE_L_SAMPLES) {
			float duty_time = BLDC_MEASURE_L_DUTY * BLDC_DT;
			measure_l_cnt /= BLDC_MEASURE_L_WAIT_CYCLES;
			printf("Measured L AVR1 VCC[V]: %.3f\n", (double) (measure_l_vcc_avr_1 / (float) measure_l_cnt));
			printf("Measured L AVR1 I1[A]:  %.3f\n", (double) (measure_l_i1_avr_1 / (float) measure_l_cnt));
			printf("Measured L AVR1 I3[A]:  %.3f\n", (double) (measure_l_i3_avr_1 / (float) measure_l_cnt));

			printf("Measured L AVR2 VCC[V]: %.3f\n", (double) (measure_l_vcc_avr_2 / (float) measure_l_cnt));
			printf("Measured L AVR2 I1[A]:  %.3f\n", (double) (measure_l_i1_avr_2 / (float) measure_l_cnt));
			printf("Measured L AVR2 I3[A]:  %.3f\n", (double) (measure_l_i3_avr_2 / (float) measure_l_cnt));

			float i1 = (measure_l_i1_avr_1 + measure_l_i3_avr_1) / (float) measure_l_cnt;
			float i2 = (measure_l_i1_avr_2 + measure_l_i3_avr_2) / (float) measure_l_cnt;
			float v1 = measure_l_vcc_avr_1 / (float) measure_l_cnt;
			float v2 = measure_l_vcc_avr_2 / (float) measure_l_cnt;
			float l = -(i1 * v2 + (-i2 - i1) * v1) * duty_time / (2.0f * i1 * i1) * 2.0f / 3.0f * 1e6f;

			printf("Measured L AVR L[uH]:   %.3f\n", (double) l);

			bldc_set_active_state(BLDC_STATE_DO_NOTHING);
		}
	}
}

CCMRAM_FUCNTION void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, /*volatile*/float *phase) {
	float L = MOTOR_L * 3.0f / 3.0f;
	float R = MOTOR_R * 3.0f / 3.0f;
	float lambda = MOTOR_LAMBDA;

	const float L_ia = L * i_alpha;
	const float L_ib = L * i_beta;
	const float R_ia = R * i_alpha;
	const float R_ib = R * i_beta;
	const float lambda_2 = lambda * lambda;
	const float gamma_half = BLDC_OBSERVER_GAMMA * 0.5f;

	float err = lambda_2 - ((x1 - L_ia) * (x1 - L_ia) + (x2 - L_ib) * (x2 - L_ib));

	float x1_dot = -R_ia + v_alpha + gamma_half * (x1 - L_ia) * err;
	float x2_dot = -R_ib + v_beta + gamma_half * (x2 - L_ib) * err;
	x1 += x1_dot * BLDC_DT;
	x2 += x2_dot * BLDC_DT;

	*phase = fast_atan2f_sec(x2 - L_ib, x1 - L_ia);
}

CCMRAM_FUCNTION static void pll_run(float phase, float dt, float *phase_var, float *speed_var) {
	float delta_theta = phase - *phase_var;
	delta_theta = fast_norm_angle_rad(delta_theta);
	*phase_var += (*speed_var + BLDC_PLL_KP * delta_theta) * dt;
	*phase_var = fast_norm_angle_rad(*phase_var);
	*speed_var += BLDC_PLL_KI * delta_theta * dt;
}

void bldc_init(void) {
	//TODO remove compensation in future revisions
	adc_vref_mul_vrefint_cal = ADC_VREF_V * ((float) (*VREFINT_CAL)) * ADC_VREF_COMPENSATION;
	one_over_adc_temp_call = 1.0f / (float) (*ADC_TEMP110_CAL_ADDR - *ADC_TEMP30_CAL_ADDR);
	bldc_set_active_state(BLDC_STATE_CALIBRATE_I);
}

void bldc_set_i_q_ref(float iq) {
	i_q_ref_rc = iq;
}

FrameDisplayChannelsData4 * bldc_get_scope_4ch_frame(uint32_t index) {
	return (FrameDisplayChannelsData4 *) &(scope_frame_buff[index]);
}

bool bldc_get_frame_ready(uint32_t index) {
	return scope_frame_ready_buff[index];
}

void bldc_get_frame_ready_clear(uint32_t index) {
	scope_frame_buff_depth--;
	scope_frame_ready_buff[index] = false;
}

CCMRAM_FUCNTION void bldc_scope_send_data(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4) {
	static uint32_t frame_index_cnt = 0;
	static uint32_t scope_frame_data_cnt = 0;
	static uint32_t scope_frame_packet_cnt = 0;

	if (scope_frame_data_cnt < FRAME_MAX_DISPLAY_CHANNELS_8 * 2) {
		uint32_t index = scope_frame_data_cnt;
		scope_frame_buff[frame_index_cnt].ch1[index] = ch1;
		scope_frame_buff[frame_index_cnt].ch2[index] = ch2;
		scope_frame_buff[frame_index_cnt].ch3[index] = ch3;
		scope_frame_buff[frame_index_cnt].ch4[index] = ch4;

		scope_frame_data_cnt++;
		if (scope_frame_data_cnt == FRAME_MAX_DISPLAY_CHANNELS_8 * 2) {
			scope_frame_buff[frame_index_cnt].packet_cnt = scope_frame_packet_cnt;
			scope_frame_packet_cnt++;
			scope_frame_ready_buff[frame_index_cnt] = true;

			scope_frame_data_cnt = 0;

			scope_frame_buff_depth++;
			if (scope_frame_buff_depth > scope_frame_buff_max_depth) {
				scope_frame_buff_max_depth = scope_frame_buff_depth;
			}

			frame_index_cnt++;
			if (frame_index_cnt == BLDC_FRAME_SCOPE_BUFF_SIZE) {
				frame_index_cnt = 0;
			}
		}
	}
}

CCMRAM_FUCNTION void bldc_2d_saturation_limit(float *x, float *y, float max) {
	float mag;
	arm_sqrt_f32((*x) * (*x) + (*y) * (*y), &mag);

	//Prevent division by zero
	if (mag < 1e-10f) {
		mag = 1e-10f;
	}

	if (mag > max) {
		float f = max / mag;
		(*x) *= f;
		(*y) *= f;
	}
}

CCMRAM_FUCNTION static void bldc_state_foc(void) {
	//TODO add window comparator

	//Check startup speed timeout
	if (tick_get_time_ms() - foc_start_time > BLDC_START_TIEMOUT_MS) {
		if (fabsf(motor_speed_rps) < BLDC_STARTUP_SPEED_RPS) {
			bldc_set_active_state(BLDC_STATE_STOP);				//Comment when flux linkage measurements
			return;												//Comment when flux linkage measurements
		}
	}

	//Ramp i_q_ref
	if (i_q_ref > i_q_ref_rc) {
		if (i_q_ref - i_q_ref_rc > BLDC_MAX_DQ_STEP) {
			i_q_ref -= BLDC_MAX_DQ_STEP;
		} else {
			i_q_ref = i_q_ref_rc;
		}
	} else if (i_q_ref < i_q_ref_rc) {
		if (i_q_ref_rc - i_q_ref > BLDC_MAX_DQ_STEP) {
			i_q_ref += BLDC_MAX_DQ_STEP;
		} else {
			i_q_ref = i_q_ref_rc;
		}
	}

	//No ramp
	//i_q_ref = i_q_ref_rc;

	//IQ Current limit
	if (i_q_ref > i_q_max) {
		i_q_ref = i_q_max;
	} else if (i_q_ref < -i_q_max) {
		i_q_ref = -i_q_max;
	}

	//Check saturation protection
	bldc_2d_saturation_limit(&i_d_ref, &i_q_ref, i_q_max);

	//Read I1 and I3
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) * v_ldo_v / ( ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) * v_ldo_v / ( ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);

	//Clarke Transform
	i_alpha = p1_i;
	i_beta = ONE_BY_SQRT3 * p1_i + TWO_BY_SQRT3 * p3_i;

	//Run observer
	float teta_rad;

	//FOC
	observer_update(v_alpha, v_beta, i_alpha, i_beta, &teta_rad);
	tetha_deg = RAD_TO_DEG(teta_rad);								//Comment when flux linkage measurements

	//PLL
	pll_run(teta_rad, BLDC_DT, &pll_phase_rad, &pll_speed_rad);

	//Predict position in next step
	tetha_deg += RAD_TO_DEG(pll_speed_rad) * BLDC_DT;					//Comment when flux linkage measurements

	//Calculate magnetic
	magnetic_speed_rps = pll_speed_rad / (2.0f * (float) M_PI);

	//Calculate motor speed
	motor_speed_rps = magnetic_speed_rps / MOTOR_NUMER_OF_POLES;

	//Startup
	if (tick_get_time_ms() - foc_start_time < BLDC_STARTUP_TIME_MS) {
		//TODO add speed independent of the frequency
		theta_over_deg = theta_over_deg - 0.1f * SIGN(motor_speed_target_rps);
		tetha_deg = fast_norm_angle_deg(theta_over_deg);
	}

	//Park Transform
	float sin_tetha, cos_tetha;
	arm_sin_cos_f32(tetha_deg, &sin_tetha, &cos_tetha);

	float i_d = i_alpha * cos_tetha + i_beta * sin_tetha;
	float i_q = i_beta * cos_tetha - i_alpha * sin_tetha;

	//PID
	pid_d.out_limit = BLDC_PID_I_LIMIT * v_vcc_v * BLDC_MAX_DUTY;
	pid_q.out_limit = BLDC_PID_I_LIMIT * v_vcc_v * BLDC_MAX_DUTY;

	float v_d = pid_control_pi(&pid_d, i_d, i_d_ref, BLDC_DT);
	float v_q = pid_control_pi(&pid_q, i_q, i_q_ref, BLDC_DT);

	//Maximum limitation
	bldc_2d_saturation_limit(&v_d, &v_q, BLDC_VDQ_MAX_LIMIT * v_vcc_v * BLDC_MAX_DUTY);

	//Calculate Vdq
	float v_mod = (2.0f / 3.0f) * v_vcc_v;
	float v_mod_inv = 1.0f / v_mod;
	float mod_d = v_d * v_mod_inv;
	float mod_q = v_q * v_mod_inv;

	//Inverse Park
	mod_alpha = cos_tetha * mod_d - sin_tetha * mod_q;
	mod_beta = cos_tetha * mod_q + sin_tetha * mod_d;

	//Dead time compensation
	float i_alpha_f = cos_tetha * i_d_ref - sin_tetha * i_q_ref;
	float i_beta_filter = cos_tetha * i_q_ref + sin_tetha * i_d_ref;

	float ia_filter = i_alpha_f;
	float ib_filter = -0.5f * i_alpha_f + SQRT3_BY_2 * i_beta_filter;
	float ic_filter = -0.5f * i_alpha_f - SQRT3_BY_2 * i_beta_filter;

	float mod_alpha_f_sgn = (2.0f / 3.0f) * SIGN(ia_filter) - (1.0f / 3.0f) * SIGN(ib_filter) - (1.0f / 3.0f) * SIGN(ic_filter);
	float mod_beta_f_sgn = ONE_BY_SQRT3 * SIGN(ib_filter) - ONE_BY_SQRT3 * SIGN(ic_filter);

	float mod_comp_fact = DRV8301_PWM_3F_DEAD_TIME_S * (float) DRV8301_PWM_3F_SWITCHING_FREQ_HZ;

	//Alpha beta to voltage
	v_alpha = mod_alpha * v_mod;
	v_beta = mod_beta * v_mod;

	//Correct dead time
	mod_alpha += mod_alpha_f_sgn * mod_comp_fact;
	mod_beta += mod_beta_f_sgn * mod_comp_fact;

	//SVM
	uint32_t duty1, duty2, duty3, svm_sector;
	bldc_svm(mod_alpha, mod_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);

	//FLux linkage measurements
	//bldc_flux_linkage_measurement(v_d, v_q, i_d, i_q);

	//Set PWM
	DRV8301_PWM_UPDATE_DISABLE;
	drv8301_set_pwm(duty1, duty2, duty3);
	DRV8301_PWM_UPDATE_ENABLE;

	//Speed controller
	i_q_ref_rc = pid_control_pid(&pid_speed, motor_speed_rps, speed_direction * motor_speed_target_rps, BLDC_DT);

	//Send to scope
	float ch1 = RAD_TO_DEG(teta_rad) * 10.0f;
	float ch2 = motor_speed_rps * 10.0f;
	float ch3 = motor_speed_target_rps * 10.0f;
	float ch4 = p3_i * 1000.0f;

	//bldc_scope_send_data((int16_t) ch1, (int16_t) ch2, (int16_t) ch3, (int16_t) ch4);
}

CCMRAM_FUCNTION void bldc_adc_irq_hanlder(void) {
	//V LDO calculate
	float tmp = adc_vref_mul_vrefint_cal / ((float) ADC_INJ_VREF_INT);
	v_ldo_v = v_ldo_v * (1.0f - BLDC_LDO_VCC_LPF_ALPHA) + tmp * BLDC_LDO_VCC_LPF_ALPHA;

	//Calculate NTC temperature
	tmp = (ADC_NTC_R2_OHM * ((float) ADC_INJ_NTC)) / (ADC_MAX_VALUE - ((float) ADC_INJ_NTC));
	ntc_temperature_c = ADC_NTC_B_25_100_K / fast_log(tmp / ADC_NTC_R_INF) - ADC_KELVIN_OFFSET;

	//Calculate uP temperature
	tmp = (((float) ADC_INJ_TEMP_SENS) - (float) *ADC_TEMP30_CAL_ADDR) * (110.0f - 30.0f);
	up_temperature_c = tmp * one_over_adc_temp_call + 30.0f;

	//Voltage calculation
	tmp = ((float) ADC_INJ_VCC) * v_ldo_v * (ADC_V_GAIN / ADC_MAX_VALUE);
	v_vcc_v = v_vcc_v * (1.0f - BLDC_LDO_VCC_LPF_ALPHA) + tmp * BLDC_LDO_VCC_LPF_ALPHA;

	bldc_active_state_cb();
}

static void bldc_state_calibrate_finish(void) {
	if (!drv8301_get_i_calibration_status()) {
		bldc_state_stop();
		bldc_set_active_state(BLDC_STATE_STOP);
	}
}

void bldc_increase_motor_speed_rps(float speed_rps) {
	if (bldc_active_state == BLDC_STATE_FOC) {
		motor_speed_target_rps += speed_rps;
		//TODO implement speed max limit
		if (fabsf(motor_speed_target_rps) < MOTOR_MIN_SPEED_TARGET_RPS) {
			bldc_set_active_state(BLDC_STATE_STOP);
		}
	}
}

void bldc_stop_sig(void) {
	if (bldc_active_state == BLDC_STATE_FOC) {
		bldc_set_active_state(BLDC_STATE_STOP);
	}
}

void bldc_start_sig(void) {
	if (bldc_active_state == BLDC_STATE_DO_NOTHING) {
		//Initialize all FOC parameters

		//Startup timeout
		foc_start_time = tick_get_time_ms();

		//Set current limit
		i_q_max = BLDC_IQ_MAX_FOR_RUN;

		//Set target IQ
		i_q_ref = 0.0f;

		//Start motor speed target
		motor_speed_target_rps = MOTOR_MIN_SPEED_TARGET_RPS;

		//Clear all integral components
		//Current control PID
		i_d_err_acc = 0;
		i_q_err_acc = 0;

		//PLL
		pll_phase_rad = 0;
		pll_speed_rad = 0;

		//PID
		pid_reset(&pid_speed);
		pid_reset(&pid_d);
		pid_reset(&pid_q);

		//Observer
		x1 = 0;
		x2 = 0;

		//Theta
		theta_over_deg = 0;

		//Set this to run FOC
		bldc_enable_all_pwm_output();
		DRV8301_PWM_UPDATE_EVENT;
		bldc_set_active_state(BLDC_STATE_FOC);

		//Set this to run R measurement
		//Hold the motor to not rotate, remember to run this measurement 2,3x to increase the motor temperature
		//bldc_measure_r_init();

		//Set this to run L measurement
		//bldc_measure_l_init();
	}
}

static void bldc_state_do_nothing(void) {
	//TODO consider disable the IRQ or reduce the switching frequency down to 1k
}

void bldc_set_active_state(BldcStateMachine state) {
	bldc_active_state = state;
	bldc_active_state_cb = state_dictionary[state].state_cb;
}

static void bldc_flux_linkage_measurement(float v_d, float v_q, float i_d, float i_q) {
	//Two different methods of measurements
	i_q_ref_rc = 3.0f;

	static float add_min_speed = 0;
	static float tetha_start = 0;
	uint32_t measurement_start_time_ms = 13000;
	uint32_t measurement_time_ms = 2000;

	if (tick_get_time_ms() < measurement_start_time_ms + measurement_time_ms * 2) {
		if (tick_get_time_ms() < measurement_start_time_ms - measurement_time_ms) {
			static uint16_t zero_time = 0;
			if (zero_time == 0) {
				zero_time = tick_get_time_ms();
			}
			uint32_t time = tick_get_time_ms() - zero_time;

			float t = time * time / 4000.0f;
			add_min_speed = (t * 2.0f * (float) M_PI) / 60.0f;
		}
		tetha_start += RAD_TO_DEG(add_min_speed) * BLDC_DT;

		if (tetha_start < 180.0f) {
			tetha_start += 360.0f;
		}

		if (tetha_start > 180.0f) {
			tetha_start -= 360.0f;
		}
		tetha_deg = tetha_start;

	}

	static float vq_avg = 0.0;
	static float vd_avg = 0.0;
	static float iq_avg = 0.0;
	static float id_avg = 0.0;
	static float samples2 = 0.0;

	static bool finished = false;
	static bool bldc_off = false;

	static float va_bemf_max = 0.0;
	static float vb_bemf_max = 0.0;
	static float vc_bemf_max = 0.0;

	if (bldc_off) {
		float v1 = ADC_INJ_P1_BEMF * ADC_V_GAIN;
		v1 *= v_ldo_v / ADC_MAX_VALUE;

		float v2 = ADC_INJ_P2_BEMF * ADC_V_GAIN;
		v2 *= v_ldo_v / ADC_MAX_VALUE;

		float v3 = ADC_INJ_P3_BEMF * ADC_V_GAIN;
		v3 *= v_ldo_v / ADC_MAX_VALUE;

		float vdc = (v1 + v2 + v3) / 3.0f;

		v1 -= vdc;
		v2 -= vdc;
		v3 -= vdc;

		v1 = fabsf(v1);
		v2 = fabsf(v2);
		v3 = fabsf(v3);

		if (v1 > va_bemf_max) {
			va_bemf_max = v1;
		}

		if (v2 > va_bemf_max) {
			vb_bemf_max = v2;
		}

		if (v3 > va_bemf_max) {
			vc_bemf_max = v3;
		}

		linkage_bemf_a = va_bemf_max / add_min_speed;
		linkage_bemf_b = vb_bemf_max / add_min_speed;
		linkage_bemf_c = vc_bemf_max / add_min_speed;

		//TODO replace to generate this info from main
		//printf("Flux linkage VBMF/Speed A: %.6f\n", (double) linkage_bemf_a);
		//printf("Flux linkage VBMF/Speed B: %.6f\n", (double) linkage_bemf_b);
		//printf("Flux linkage VBMF/Speed C: %.6f\n", (double) linkage_bemf_c);
	}

	if (tick_get_time_ms() == (measurement_start_time_ms + measurement_time_ms) && finished == false) {
		finished = true;

		vq_avg /= samples2;
		vd_avg /= samples2;
		iq_avg /= samples2;
		id_avg /= samples2;

		linkage = (sqrtf(vq_avg * vq_avg + vd_avg * vd_avg) - MOTOR_R * sqrtf(iq_avg * iq_avg + id_avg * id_avg)) / add_min_speed;

		i_q_ref_rc = 0;

		drv8301_set_pwm(0, 0, 0);

		bldc_disable_all_pwm_output();
		DRV8301_PWM_UPDATE_EVENT;	//TODO verify if required
		bldc_off = true;

		//TODO replace to generate this info from main
		//printf("Flux linkage V/I: %.6f\n", (double) linkage);

	} else if (tick_get_time_ms() > measurement_start_time_ms && finished == false) {
		vq_avg += v_q;
		vd_avg += v_d;
		iq_avg += i_q;
		id_avg += i_d;
		samples2 += 1.0f;
	}
}
