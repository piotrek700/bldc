#include "bldc.h"
#include "../Frame/frame_frames.h"
#include "utils.h"

//Calibration
#define BEMF_V_CALIBRATION_WAIT_TIME_MS			10

#define BLDC_R_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_R_SAMPLES					1024*8
#define BLDC_MEASURE_R_THETA_DEG				150.0f
#define BLDC_MEASURE_R_CURRENT_A				5.0f

#define BLDC_L_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_L_WAIT_CYCLES				64
#define BLDC_MEASURE_L_DUTY						0.4f
#define BLDC_MEASURE_L_SAMPLES					1024*BLDC_MEASURE_L_WAIT_CYCLES
#define BLDC_MAX_DUTY							0.8f	//todo can be increased

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

static float p1_i_offset = 0;
static float p3_i_offset = 0;
static float i_offset_ldo = 0;

#define ONE_BY_SQRT3							0.57735026919f
#define TWO_BY_SQRT3							1.15470053838f
#define RAD_TO_DEG								57.2957795131f
#define ONE_OVER_3							    0.66666666667f


//MOTO
#define MOTOR_R									(0.084f * (3.0f/3.0f))			//TODO sprawdcz czy mnozyc czy nie 0.088
#define MOTOR_L									(9.27e-6f * (3.0f/3.0f))		//TODO sprawdcz czy mnozyc czy nie 9.27e-6f
#define MOTOR_LAMBDA							0.000506f//0.000609f						//TODO sprawdcz czy mnozyc czy nie 0.000609f
#define MOTOR_PID_TIME_CONSTANT					0.001f

#define MOTOR_KA								(MOTOR_L/MOTOR_PID_TIME_CONSTANT)
#define MOTOR_KB								(MOTOR_R/MOTOR_L)
#define SQRT3_BY_2								0.86602540378f
#define SQRT3_BY_2_MUL_2_OVER_3					0.57735026919f

//FOC
#define BLDC_DQ_LPF_CUTOFF_FREQ					450.0f	//2000
#define BLDC_DT									(1.0f/(float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)
#define BLDC_VDQ_MAX_LIMIT						ONE_BY_SQRT3 	//(SQRT3_BY_2 * 2.0f / 3.0f)

//#define BLDC_PID_KP 							MOTOR_KA					//0.2
//#define BLDC_PID_KI								MOTOR_KB * MOTOR_KA		//100

//Speed PID
#define BLDC_SPEED_PID_KP						0.005f
#define BLDC_SPEED_PID_KI						0.0f
#define BLDC_SPEED_PID_KD						0.0f
#define BLDC_SPEED_PID_I_LIMIT					1.0f


volatile float BLDC_PID_KP 					=		MOTOR_KA;
volatile float BLDC_PID_KI					=			(MOTOR_KB* MOTOR_KA);
#define BLDC_PID_I_LIMIT						BLDC_VDQ_MAX_LIMIT
#define BLDC_PID_OUT_LIMIT 						BLDC_VDQ_MAX_LIMIT


float tetha = 0;

float i_d_ref = 0;
float i_q_ref = 0.0;		//2
float i_q_max = 15.0;
float i_d_err_acc = 0;
float i_q_err_acc = 0;
float i_d_lpf = 0;
float i_q_lpf = 0;

float speed_err_acc = 0;

#define BLDC_DQ_LPF_ALPHA						(BLDC_DQ_LPF_CUTOFF_FREQ/(BLDC_DQ_LPF_CUTOFF_FREQ+((float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)/(2.0f*(float)M_PI)))

//HFI
#define BLDC_HFI_LPF_CUTOFF_FREQ				5000.0f
#define BLDC_HFI_LPF_ALPHA						(BLDC_HFI_LPF_CUTOFF_FREQ/(BLDC_HFI_LPF_CUTOFF_FREQ+((float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)/(2.0f*(float)M_PI)))


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

static /*volatile*/ float v_vcc_v = 0;
static /*volatile*/ float v_ldo_v = 0;
static /*volatile*/ float up_temperature_c = 0;
static /*volatile*/ float ntc_temperature_c = 0;

static void bldc_state_calibrate_i(void);
static void bldc_state_calibrate_v(void);
static void bldc_state_calibrate_finish(void);
static void bldc_state_measure_r(void);
static void bldc_state_measure_l(void);
static void bldc_state_stop(void);
static void bldc_state_foc(void);
static void bldc_state_do_nothing(void);

static BldcStateDictionaryRow state_dictionary[] = {
		{BLDC_STATE_CALIBRATE_I, 		bldc_state_calibrate_i},
		{BLDC_STATE_CALIBRATE_V,  		bldc_state_calibrate_v},
		{BLDC_STATE_CALIBRATE_FINISH, 	bldc_state_calibrate_finish},
		{BLDC_STATE_MEASURE_R,			bldc_state_measure_r},
		{BLDC_STATE_MEASURE_L,			bldc_state_measure_l},
		{BLDC_STATE_STOP,				bldc_state_stop},
		{BLDC_STATE_FOC,				bldc_state_foc},
		{BLDC_STATE_DO_NOTHING,			bldc_state_do_nothing}
};

static void (*bldc_active_state_cb)(void) = bldc_state_do_nothing;
static BldcStateMachine bldc_active_state = BLDC_STATE_DO_NOTHING;

float m_gamma_now = 2696282635.13f; //* 0.3f;//5e8;		//bw/(lambda * lambda) = 10000/

static float x1 = 0;
static float x2 = 0;

/*volatile*/ float m_pll_phase = 0;
/*volatile*/ float m_pll_speed = 0;

volatile float foc_pll_kp = 2000.0;
volatile float foc_pll_ki = 40000.0;		//1000000

static volatile float g_i_abs=0;
static volatile float g_i_d=0;
static volatile float g_i_q=0;
static volatile float g_v_d=0;
static volatile float g_v_q=0;

static float adc_vref_mul_vrefint_cal = 0;
static float one_over_adc_temp_call = 0;

void bldc_set_i_d(float i_d){
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

static void bldc_svm(float alpha, float beta, uint32_t PWMHalfPeriod, uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
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

	// PWM timings
	uint32_t tA, tB, tC;

	switch (sector) {

	// sector 1-2
	case 1: {
		// Vector on-times
		uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t1 - t2) / 2;
		tB = tA + t1;
		tC = tB + t2;

		break;
	}

		// sector 2-3
	case 2: {
		// Vector on-times
		uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t2 - t3) / 2;
		tA = tB + t3;
		tC = tA + t2;

		break;
	}

		// sector 3-4
	case 3: {
		// Vector on-times
		uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t3 - t4) / 2;
		tC = tB + t3;
		tA = tC + t4;

		break;
	}

		// sector 4-5
	case 4: {
		// Vector on-times
		uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t4 - t5) / 2;
		tB = tC + t5;
		tA = tB + t4;

		break;
	}

		// sector 5-6
	case 5: {
		// Vector on-times
		uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t5 - t6) / 2;
		tA = tC + t5;
		tB = tA + t6;

		break;
	}

		// sector 6-1
	case 6: {
		// Vector on-times
		uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t6 - t1) / 2;
		tC = tA + t1;
		tB = tC + t6;

		break;
	}
	}

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
	*svm_sector = sector;
}

static void bldc_enable_all_pwm_output(void){
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

static void bldc_stop_pwm(void){
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
	} else if (tick_get_time_ms() - time > BEMF_V_CALIBRATION_WAIT_TIME_MS) {	//TODO define
		if (h_complete) {
			if (p_offset_cnt < ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_offset += ADC_INJ_P1_BEMF;
				v2_bemf_offset += ADC_INJ_P2_BEMF;
				v3_bemf_offset += ADC_INJ_P3_BEMF;
				v_bemf_offset_ldo += v_ldo_v;
				v_bemf_offset_vcc += v_vcc_v;

				p_offset_cnt++;
			} else if (p_offset_cnt == ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_offset /= (float) (p_offset_cnt - 1);
				v2_bemf_offset /= (float) (p_offset_cnt - 1);
				v3_bemf_offset /= (float) (p_offset_cnt - 1);
				v_bemf_offset_ldo /= (float) (p_offset_cnt - 1);
				v_bemf_offset_vcc /= (float) (p_offset_cnt - 1);

				float v1 = v1_bemf_offset * ADC_V_GAIN;
				v1 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				float v2 = v2_bemf_offset * ADC_V_GAIN;
				v2 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				float v3 = v3_bemf_offset * ADC_V_GAIN;
				v3 *= v_bemf_offset_ldo / ADC_MAX_VALUE;

				//GND floor measurement
				printf("Calibration BEMF L V1[V]: %f\n", v1);
				printf("Calibration BEMF L V2[V]: %f\n", v2);
				printf("Calibration BEMF L V3[V]: %f\n", v3);
				printf("Calibration BEMF L VLDO[V]: %f\n", v_bemf_offset_ldo);
				printf("Calibration BEMF L VCC[V]: %f\n", v_bemf_offset_vcc);

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
				v1_bemf_h /= (float) (p_offset_cnt - 1);
				v2_bemf_h /= (float) (p_offset_cnt - 1);
				v3_bemf_h /= (float) (p_offset_cnt - 1);
				v_bemf_h_ldo /= (float) (p_offset_cnt - 1);
				v_bemf_h_vcc /= (float) (p_offset_cnt - 1);

				float v1 = v1_bemf_h * ADC_V_GAIN;
				v1 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				float v2 = v2_bemf_h * ADC_V_GAIN;
				v2 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				float v3 = v3_bemf_h * ADC_V_GAIN;
				v3 *= v_bemf_h_ldo / ADC_MAX_VALUE;

				//VCC measurement
				printf("Calibration BEMF H V1[V]: %f\n", v1);
				printf("Calibration BEMF H V2[V]: %f\n", v2);
				printf("Calibration BEMF H V3[V]: %f\n", v3);
				printf("Calibration BEMF H VLDO[V]: %f\n", v_bemf_h_ldo);
				printf("Calibration BEMF H VCC[V]: %f\n", v_bemf_h_vcc);

				time = 0;
				h_complete = true;
			}
		}
	}
}

static void bldc_state_calibrate_i(void) {
	//Calibrate
	static uint32_t p_offset_cnt = 0;

	if (p_offset_cnt < ADC_I_OFFSET_COUNTER_MAX) {
		p1_i_offset += ADC_INJ_P1_I;
		p3_i_offset += ADC_INJ_P3_I;
		i_offset_ldo += v_ldo_v;
		p_offset_cnt++;
	} else if (p_offset_cnt == ADC_I_OFFSET_COUNTER_MAX) {
		p1_i_offset /= (float) (p_offset_cnt - 1);
		p3_i_offset /= (float) (p_offset_cnt - 1);
		i_offset_ldo /= (float) (p_offset_cnt - 1);

		float p1 = ((float) p1_i_offset - ADC_MAX_VALUE / 2) / ADC_I_GAIN;
		p1 *= i_offset_ldo / ADC_MAX_VALUE / ADC_I_R_OHM;

		float p3 = ((float) p3_i_offset - ADC_MAX_VALUE / 2) / ADC_I_GAIN;
		p3 *= i_offset_ldo / ADC_MAX_VALUE / ADC_I_R_OHM;

		printf("Calibration I1[A]: %f\n", p1);
		printf("Calibration I3[A]: %f\n", p3);
		printf("Calibration I VLDO[V]: %f\n", i_offset_ldo);

		drv8301_i_calibration_disable();
		bldc_set_active_state(BLDC_STATE_CALIBRATE_V);
	}
}

static void bldc_state_stop(void) {
	//TODO optimize and verify - stop should open all keys or close?
	drv8301_set_pwm(0, 0, 0);

	bldc_stop_pwm();
	DRV8301_PWM_UPDATE_EVENT;

	bldc_set_active_state(BLDC_STATE_DO_NOTHING);
}

bool bldc_measure_r_init(void) {
	if(bldc_active_state != BLDC_STATE_DO_NOTHING){
		return false;
	}

	//Set flow VDC->L->L->I1, VDC->L->L->I2, not in reverse to avoid V diode drop
	tetha = BLDC_MEASURE_R_THETA_DEG;
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

static void bldc_state_measure_r(void) {
	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//I3
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//Clarke Transform
	float i_alpha = p1_i;
	float i_beta = ONE_BY_SQRT3 * p1_i + TWO_BY_SQRT3 * p3_i;

	//Park Transform
	float sin_tetha;
	float cos_tetha;
	arm_sin_cos_f32(tetha, &sin_tetha, &cos_tetha);

	float i_d = i_alpha * cos_tetha + i_beta * sin_tetha;
	float i_q = i_beta * cos_tetha - i_alpha * sin_tetha;

	//Current LPF
	i_d_lpf = BLDC_DQ_LPF_ALPHA * i_d + (1.0f - BLDC_DQ_LPF_ALPHA) * i_d_lpf;
	i_q_lpf = BLDC_DQ_LPF_ALPHA * i_q + (1.0f - BLDC_DQ_LPF_ALPHA) * i_q_lpf;

	i_d = i_d_lpf;
	i_q = i_q_lpf;

	//PID P error
	float i_d_err = i_d_ref - i_d;
	float i_q_err = i_q_ref - i_q;

	//PID I error
	i_d_err_acc += i_d_err * (BLDC_DT * BLDC_PID_KI);
	i_q_err_acc += i_q_err * (BLDC_DT * BLDC_PID_KI);

	//PID I limit
	if (i_d_err_acc > BLDC_PID_I_LIMIT * v_vcc_v) {
		i_d_err_acc = BLDC_PID_I_LIMIT * v_vcc_v;
	} else if (i_d_err_acc < -BLDC_PID_I_LIMIT * v_vcc_v) {
		i_d_err_acc = -BLDC_PID_I_LIMIT * v_vcc_v;
	}

	if (i_q_err_acc > BLDC_PID_I_LIMIT * v_vcc_v) {
		i_q_err_acc = BLDC_PID_I_LIMIT * v_vcc_v;
	} else if (i_q_err_acc < -BLDC_PID_I_LIMIT * v_vcc_v) {
		i_q_err_acc = -BLDC_PID_I_LIMIT * v_vcc_v;
	}

	//PID out limit
	float v_d = i_d_err * BLDC_PID_KP + i_d_err_acc;
	float v_q = i_q_err * BLDC_PID_KP + i_q_err_acc;

	//TODO dynamic I limit

	//Maximum limitation
	float v_dq_mag = sqrtf(v_d * v_d + v_q * v_q);
	float v_dq_max = BLDC_VDQ_MAX_LIMIT * v_vcc_v;

	if (v_dq_mag > v_dq_max) {
		float dq_scale = v_dq_max / v_dq_mag;
		v_d *= dq_scale;
		v_q *= dq_scale;
	}

	float v_d2 = v_d / (v_vcc_v * 2.0f / 3.0f);
	float v_q2 = v_q / (v_vcc_v * 2.0f / 3.0f);

	//Inverse Park
	float v_alpha = v_d2 * cos_tetha - v_q2 * sin_tetha;
	float v_beta = v_q2 * cos_tetha + v_d2 * sin_tetha;

	//TODO dead time compensation

	//SVM
	uint32_t duty1, duty2, duty3, svm_sector;
	bldc_svm(v_alpha, v_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);

	drv8301_set_pwm(duty1, duty2, duty3);
	DRV8301_PWM_UPDATE_EVENT;

	if (tick_get_time_ms() - meaure_r_start_time > BLDC_R_MEASUREMENT_START_TIME_MS) {
		float i_dc_link = sqrtf(i_d * i_d + i_q * i_q);
		measure_r_current_avr += i_dc_link;

		float v_dc_link = sqrtf(v_d * v_d + v_q * v_q);
		measure_r_voltage_avr += v_dc_link;

		measure_r_cnt++;

		if (measure_r_cnt == BLDC_MEASURE_R_SAMPLES) {
			printf("Measured R AVR Vdq[V]: %f\n", measure_r_voltage_avr / (float) measure_r_cnt);
			printf("Measured R AVR Idq[A]: %f\n", measure_r_current_avr / (float) measure_r_cnt);
			printf("Measured R AVR R[Ohm]: %f\n", measure_r_voltage_avr / measure_r_current_avr);

			bldc_set_active_state(BLDC_STATE_STOP);
		}
	}
}

bool bldc_measure_l_init(void) {
	if(bldc_active_state != BLDC_STATE_DO_NOTHING){
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

	float duty = BLDC_MEASURE_L_DUTY;

	if (tick_get_time_ms() - meaure_l_start_time > BLDC_L_MEASUREMENT_START_TIME_MS) {

		if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 0) {
			//drv8301_set_pwm(0, duty * DRV8301_PWM_3F_PWM_MAX,0);
			drv8301_set_pwm(duty * DRV8301_PWM_3F_PWM_MAX, 0, duty * DRV8301_PWM_3F_PWM_MAX);
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
			float duty_time = duty * BLDC_DT;
			measure_l_cnt /= BLDC_MEASURE_L_WAIT_CYCLES;
			printf("Measured L AVR1 VCC[V]: %f\n", measure_l_vcc_avr_1 / (float) measure_l_cnt);
			printf("Measured L AVR1 I1[A]: %f\n", measure_l_i1_avr_1 / (float) measure_l_cnt);
			printf("Measured L AVR1 I3[A]: %f\n", measure_l_i3_avr_1 / (float) measure_l_cnt);

			printf("Measured L AVR2 VCC[V]: %f\n", measure_l_vcc_avr_2 / (float) measure_l_cnt);
			printf("Measured L AVR2 I1[A]: %f\n", measure_l_i1_avr_2 / (float) measure_l_cnt);
			printf("Measured L AVR2 I3[A]: %f\n", measure_l_i3_avr_2 / (float) measure_l_cnt);

			float i1 = (measure_l_i1_avr_1 + measure_l_i3_avr_1) / (float) measure_l_cnt;
			float i2 = (measure_l_i1_avr_2 + measure_l_i3_avr_2) / (float) measure_l_cnt;
			float v1 = measure_l_vcc_avr_1 / (float) measure_l_cnt;
			float v2 = measure_l_vcc_avr_2 / (float) measure_l_cnt;
			float l = -(i1 * v2 + (-i2 - i1) * v1) * duty_time / (2.0f * i1 * i1) * 2.0f / 3.0f * 1e6f;

			printf("Measured L AVR L[uH]: %f\n", l);

			bldc_set_active_state(BLDC_STATE_DO_NOTHING);
		}
	}
}

void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, /*volatile*/float *phase) {
	float L = MOTOR_L * 3.0f / 3.0f;
	float R = MOTOR_R * 3.0f / 3.0f;
	float lambda = MOTOR_LAMBDA;

	const float L_ia = L * i_alpha;
	const float L_ib = L * i_beta;
	const float R_ia = R * i_alpha;
	const float R_ib = R * i_beta;
	const float lambda_2 = lambda * lambda;
	const float gamma_half = m_gamma_now * 0.5f;

	// Same as above, but without iterations.
	float err = lambda_2 - ((x1 - L_ia)*(x1 - L_ia) + (x2 - L_ib)*(x2 - L_ib));
	float gamma_tmp = gamma_half;
	float max = lambda_2 * 0.2f;

	if (err > max) {
		err = max;
		gamma_tmp *= 10.0f;
	} else if (err < -max) {
		err = -max;
		gamma_tmp *= 10.0f;
	}

	float x1_dot = -R_ia + v_alpha + gamma_tmp * (x1 - L_ia) * err;
	float x2_dot = -R_ib + v_beta  + gamma_tmp * (x2 - L_ib) * err;
	x1 += x1_dot * BLDC_DT;
	x2 += x2_dot * BLDC_DT;

	//TODO update
	if(IS_NAN(x1)){
		x1=0.0f;
	}

	if(IS_NAN(x2)){
		x2=0.0f;
	}

	*phase = fast_atan2f_sec(x2 - L_ib, x1 - L_ia);
}

void utils_norm_angle_rad(float *angle) {
	while (*angle < -(float)M_PI) {
		*angle += 2.0f * (float)M_PI;
	}

	while (*angle > (float)M_PI) {
		*angle -= 2.0f * (float)M_PI;
	}
}

static void pll_run(float phase, float dt,  float *phase_var,  float *speed_var) {
	if(IS_NAN(*phase_var)){
		*phase_var=0.0f;
	}

	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	if(IS_NAN(*speed_var)){
		*speed_var=0.0f;
	}
	*phase_var += (*speed_var + foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*) phase_var);
	*speed_var += foc_pll_ki * delta_theta * dt;
}

void bldc_init(void){
	adc_vref_mul_vrefint_cal =  ADC_VREF_V * ((float) (*VREFINT_CAL));
	one_over_adc_temp_call = 1.0f / (float) (*ADC_TEMP110_CAL_ADDR - *ADC_TEMP30_CAL_ADDR);
	bldc_set_active_state(BLDC_STATE_CALIBRATE_I);
}

volatile static float w1 = 0;
volatile static float theta_est = 0;
volatile static float LAMBDA = 2.5f;

volatile static float WLIM = 25.0f;
volatile static float ALPHA0 = 60.0f;//0.1f* WLIM;

//HFI
volatile float i_sense_hfi_lpf = 0.0f;
volatile float BLDC_HFI_KP = 1.0f;
volatile float BLDC_HFI_KI = 0.0f;
volatile float inj_i = 0.0f;
volatile float inj_phase = 0.0f;
volatile float inj_phase_deg = 0.0f;
volatile float inj_omega = 0.0f;

volatile float xx1 = 0.0f;
volatile float xx2 = 0.0f;
volatile float xx3 = 0.0f;
volatile float xx4 = 0.0f;

volatile float   i_bus = 0;

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

volatile float MCCONF_S_PID_KP					=0.1;//0.004f;// Proportional gain
volatile float MCCONF_S_PID_KI					=0.0002;//0.004f;	// Integral gain
volatile float MCCONF_S_PID_KD					=0.0001f;	// Derivative gain
volatile float MCCONF_S_PID_KD_FILTER			=0.2f;	// Derivative filter
volatile float m_speed_pid_set=0;

float i_alpha = 0;
float i_beta = 0;
float v_alpha = 0;
float v_beta = 0;
float tetha_start = 0;
volatile float linkage = 0;
static void bldc_state_foc(void) {

	//tetha_start+=0.1f;
	//if(tetha_start>180.0f){
	//	tetha_start-=360.0f;
	//}

	//Before optimization
	//FOC Cycles Min, Max, AVR: 1802, 1924, 1863.931
	//FOC Cycles Min, Max, AVR: 1753, 1919, 1850.076
	//20kHz 15:86% free
	//10kHz 50.50% free

	//Ater optimisation
	//FOC Cycles Min, Max, AVR: 1697, 1862, 1775.677


	//From CB function
	//FOC Cycles Min, Max, AVR: 275, 305, 293.599 next

	//Empty function
	//FOC Cycles Min, Max, AVR: 295, 326, 314.599


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
	//float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	//p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) * v_ldo_v / (  ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);

	//I3
	//float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	//p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) * v_ldo_v / (  ADC_MAX_VALUE * ADC_I_R_OHM * ADC_I_GAIN);

	//Clarke Transform
	i_alpha = p1_i;
	i_beta = ONE_BY_SQRT3 * p1_i + TWO_BY_SQRT3 * p3_i;

	//Park Transform
	float sin_tetha;
	float cos_tetha;
	arm_sin_cos_f32(tetha, &sin_tetha, &cos_tetha);

	float i_d = i_alpha * cos_tetha + i_beta * sin_tetha;
	float i_q = i_beta * cos_tetha - i_alpha * sin_tetha;

	//Current LPF
	//i_d_lpf = dq_lpf_alpha * i_d + (1.0f - dq_lpf_alpha) * i_d_lpf;
	//i_q_lpf = dq_lpf_alpha * i_q + (1.0f - dq_lpf_alpha) * i_q_lpf;

	float lpf_val = 1.0f - BLDC_DQ_LPF_ALPHA;
	i_d_lpf = BLDC_DQ_LPF_ALPHA * i_d + lpf_val * i_d_lpf;
	i_q_lpf = BLDC_DQ_LPF_ALPHA * i_q + lpf_val * i_q_lpf;

	//i_d = i_d_lpf;
	//i_q = i_q_lpf;

	/*
	if (fabsf(w1) > WLIM) {
		i_d = 0;
	} else {
		if (w1 >= 0.0f) {
			i_d = i_q / LAMBDA * 1.0f;
		} else {
			i_d = i_q / LAMBDA * -1.0f;
		}
	}*/
	//PID P error
	//float i_d_err = i_d_ref - i_d;
	//float i_q_err = i_q_ref - i_q;

	float i_d_err = i_d_ref - i_d;
	float i_q_err = i_q_ref - i_q;

	float v_d = i_d_err * BLDC_PID_KP + i_d_err_acc;
	float v_q = i_q_err * BLDC_PID_KP + i_q_err_acc;


	//PID I error
		//i_d_err_acc += i_d_err * dt * BLDC_PID_KI;
		//i_q_err_acc += i_q_err * dt * BLDC_PID_KI;

	i_d_err_acc += i_d_err * BLDC_DT * BLDC_PID_KI;
	i_q_err_acc += i_q_err * BLDC_DT * BLDC_PID_KI;

	//Maximum limitation
	//float v_dq_mag = sqrtf(v_d * v_d + v_q * v_q);
	float v_dq_mag=1e-10f;
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
	//if (i_d_err_acc > BLDC_PID_I_LIMIT * v_vcc_v) {
	//	i_d_err_acc = BLDC_PID_I_LIMIT * v_vcc_v;
	//} else if (i_d_err_acc < -BLDC_PID_I_LIMIT * v_vcc_v) {
	//	i_d_err_acc = -BLDC_PID_I_LIMIT * v_vcc_v;
	//}

	//if (i_q_err_acc > BLDC_PID_I_LIMIT * v_vcc_v) {
	//	i_q_err_acc = BLDC_PID_I_LIMIT * v_vcc_v;
	//} else if (i_q_err_acc < -BLDC_PID_I_LIMIT * v_vcc_v) {
	//	i_q_err_acc = -BLDC_PID_I_LIMIT * v_vcc_v;
	//}

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
	/////////////////////////////////////////////////////////////////checked

	i_bus = mod_d * i_d + mod_q * i_q;	//TODO check

	float mod_alpha = cos_tetha * mod_d - sin_tetha * mod_q;
	float mod_beta = cos_tetha * mod_q + sin_tetha * mod_d;

	v_alpha = mod_alpha * (2.0f / 3.0f) * v_vcc_v;
	v_beta = mod_beta * (2.0f / 3.0f) * v_vcc_v;

	//TODO dead time compensation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	observer_update(v_alpha, v_beta, i_alpha, i_beta, &tetha);


	//SVM
	uint32_t duty1, duty2, duty3, svm_sector;
	bldc_svm(mod_alpha, mod_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);




	/*
	if(tick_get_time_ms()<1000*5){
		//Park Transform
		float teta0;
		float teta120;
		float teta240;
		float sin_tetha0;
		float cos_tetha0;
		float sin_tetha120;
		float cos_tetha120;
		float sin_tetha240;
		float cos_tetha240;

		teta0 = tetha_start;
		teta120 = tetha_start+120.0f;
		teta240 = tetha_start-120.0f;
		if(teta120>180.0f){
			teta120 -=360.0f;
		}

		if(teta240<-180.0f){
			teta120 +=360.0f;
		}

		arm_sin_cos_f32(teta0,  &sin_tetha0, &cos_tetha0);
		arm_sin_cos_f32(teta120, &sin_tetha120, &cos_tetha120);
		arm_sin_cos_f32(teta240, &sin_tetha240, &cos_tetha240);


		duty1=DRV8301_PWM_3F_PWM_MAX/2*(1.0f+sin_tetha0 *0.05f);
		duty2=DRV8301_PWM_3F_PWM_MAX/2*(1.0f+sin_tetha120*0.05f);
		duty3=DRV8301_PWM_3F_PWM_MAX/2*(1.0f+sin_tetha240*0.05f);
	}
	*/
	TIM1->CR1 |= TIM_CR1_UDIS;
	drv8301_set_pwm(duty1, duty2, duty3);
	TIM1->CR1 &= ~TIM_CR1_UDIS;


	pll_run(tetha, BLDC_DT, &m_pll_phase, &m_pll_speed);
	//tetha = tetha * (180.0f / (float) M_PI);

	static float theta2=0;
	static float add_min_speed = 0;
	uint32_t measurement_start_time_ms = 10000;
	uint32_t measurement_time_ms = 2000;

	if (tick_get_time_ms() < measurement_start_time_ms + measurement_time_ms * 2) {		//0.1, 0.3, 0.5
		if(tick_get_time_ms() < measurement_start_time_ms-measurement_time_ms){
			float t = tick_get_time_ms() *tick_get_time_ms() / 8000.0f;
			add_min_speed = (t * 2.0f * M_PI) / 60.0f;		//400, 500, 600
		}
		tetha_start += add_min_speed * RAD_TO_DEG * BLDC_DT;
		//tetha_start+=0.02f;
		//tetha_start += 90.0f;
		if (tetha_start < 180.0f) {
			tetha_start += 360.0f;
		}

		if (tetha_start > 180.0f) {
			tetha_start -= 360.0f;
		}
		tetha = tetha_start;

	} else {
		tetha = tetha * (180.0f / (float) M_PI);
	}

	static float vq_avg = 0.0;
	static float vd_avg = 0.0;
	static float iq_avg = 0.0;
	static float id_avg = 0.0;
	static float samples2 = 0.0;
	static bool finsihed= false;
	static bool bldc_off= false;

	static float va_bemf_max = 0.0;
	static float vb_bemf_max = 0.0;
	static float vc_bemf_max = 0.0;

	if(bldc_off){
		float v1 = ADC_INJ_P1_BEMF * ADC_V_GAIN;
		v1 *= v_ldo_v / ADC_MAX_VALUE;

		float v2 = ADC_INJ_P2_BEMF * ADC_V_GAIN;
		v2 *= v_ldo_v / ADC_MAX_VALUE;

		float v3 = ADC_INJ_P3_BEMF * ADC_V_GAIN;
		v3 *= v_ldo_v / ADC_MAX_VALUE;

		float vdc = (v1+v2+v3)/3.0f;

	}


	if(tick_get_time_ms() == (measurement_start_time_ms + measurement_time_ms) && finsihed == false){
		finsihed = true;
		vq_avg /= samples2;
		vd_avg /= samples2;
		iq_avg /= samples2;
		id_avg /= samples2;

		linkage = (sqrtf(vq_avg*vq_avg + vd_avg*vd_avg) - MOTOR_R *sqrtf(iq_avg*iq_avg + id_avg*id_avg)) / add_min_speed;// * ((2.0f * (float)M_PI) / 60.0f));

		i_q_ref = 0;

		drv8301_set_pwm(0, 0, 0);

		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		DRV8301_PWM_UPDATE_EVENT;	//TODO veryfi if required
		bldc_off = true;

	}else if(tick_get_time_ms() > measurement_start_time_ms && finsihed == false){
		vq_avg+=v_q;
		vd_avg+=v_d;
		iq_avg+=i_q;
		id_avg+=i_d;
		samples2 +=1.0f;
	}

		/*
		if(fabsf(i_q_ref)<0.01f){
			drv8301_set_pwm(0, 0, 0);

		}else{
			drv8301_set_pwm(duty1, duty2, duty3);

		}*/
		//DRV8301_PWM_UPDATE_EVENT;	//TODO veryfi if required



/*

		/////////////////////////////////////////////////////////////////checked
		static float i_term = 0.0;
		static float prev_error = 0.0;
		float p_term;
		float d_term;

		//reverse direction after 10s


//		if (tick_get_time_ms() < 10 * 1000) {
//			m_speed_pid_set_rpm = 1000.0f;
//		} else {
//			m_speed_pid_set_rpm = -1000.0f;
//		}

		float m_speed_pid_set_rpm =m_speed_pid_set/ ((2.0f * (float)M_PI) / 60.0f);

		float rpm = m_pll_speed / ((2.0f * (float)M_PI) / 60.0f);
		float error = m_speed_pid_set_rpm - rpm;

	//		// Too low RPM set. Reset state and return.
	//		if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
	//			i_term = 0.0;
	//			prev_error = error;
	//			return;
	//		}


	#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))

		// Compute parameters
		p_term = error * MCCONF_S_PID_KP * (1.0f / 20.0f);
		i_term += error * MCCONF_S_PID_KI * BLDC_DT * (1.0f / 20.0f);
		d_term = (error - prev_error) * (MCCONF_S_PID_KD/BLDC_DT) * (1.0f / 20.0f);

		// Filter D
		static float d_filter = 0.0;
		UTILS_LP_FAST(d_filter, d_term, MCCONF_S_PID_KD_FILTER);
		d_term = d_filter;

		// I-term wind-up protection
		utils_truncate_number(&i_term, -1.0, 1.0);

		// Store previous error
		prev_error = error;

		// Calculate output
		float output = p_term + i_term + d_term;
		utils_truncate_number(&output, -1.0, 1.0);

		i_q_ref = output * 10.0f;
*/
		// Speed PID parameters

	static uint32_t cnt = 0;
	cnt++;
	if (cnt == 10000 / 100) {
		cnt = 0;
		xx1 = m_pll_speed;
		xx2 = i_q_ref;
		xx3 = m_pll_phase;
		xx4 = tetha;
	}


//	//HFI
//	const float BLDC_HFI_V=0.2f;
//	const float BLDC_HFI_F=5000.0f;
//	static float hfi_t=0;
//	hfi_t +=BLDC_DT;
//	//if(hfi_t >(float)M_PI/(2.0f * (float)M_PI * BLDC_HFI_F)){
//	//	hfi_t-= 2.0f*(float)M_PI/(2.0f * (float)M_PI * BLDC_HFI_F);
//	//}
//
//	if(hfi_t >1.0f/(2.0f * BLDC_HFI_F)){
//		hfi_t-= 1.0f/(BLDC_HFI_F);
//	}
//	static float sin_inj;
//	static float cos_inj;


//	//Inject
//	v_d += BLDC_HFI_V * cos_inj;
//
//	//First calculate sin then inject
//	arm_sin_cos_f32(2.0f * (float)M_PI * BLDC_HFI_F * hfi_t *RAD_TO_DEG, &sin_inj, &cos_inj);
//
//	//Sense from prevoius loop
//	i_sense_hfi_lpf = BLDC_HFI_LPF_ALPHA * i_q * sin_inj + (1.0f - BLDC_HFI_LPF_ALPHA) * i_sense_hfi_lpf;
//	i_sense_hfi_lpf = i_q;
//
//	inj_i += i_sense_hfi_lpf * BLDC_DT;
//	float tmp = i_sense_hfi_lpf * BLDC_HFI_KP + inj_i * BLDC_HFI_KI;
//	inj_phase += tmp;
//
//	while (inj_phase > (float) M_PI) {
//		inj_phase -= 2.0f * (float) M_PI;
//	}
//	while (inj_phase < -(float) M_PI) {
//		inj_phase += 2.0f * (float) M_PI;
//	}
//
//	tetha = inj_phase;
//		pll_run(tetha, BLDC_DT, &m_pll_phase, &m_pll_speed);
//
//	static uint32_t cnt =0;
//	cnt ++;
//	if(cnt == 25000/100){
//		cnt =0;
//		xx1=tetha;
//		xx2=m_pll_phase* RAD_TO_DEG;
//		xx3=m_pll_speed;
//		xx4+=m_pll_speed * BLDC_DT* RAD_TO_DEG;
//	}




	//Inject pulse

	//static uint32_t pulse_cnt = 0;
	//pulse_cnt++;

	//if(pulse_cnt< 5){
	//	v_d += BLDC_VDQ_MAX_LIMIT / 8.0f;
	//}

	//if(pulse_cnt>DRV8301_PWM_3F_SWITCHING_FREQ_HZ / 250){ //TODO enum 250hz
	//	pulse_cnt=0;
	//}

	//TODO dynamic I limit



	//float inv_vcc_mul_2_over_3 = 1.0f / (v_vcc_v * 2.0f / 3.0f);
//	float inv_vcc_mul_2_over_3 = 1.0f / (v_vcc_v * ONE_OVER_3);
//	float v_d2 = v_d * inv_vcc_mul_2_over_3;
//	float v_q2 = v_q * inv_vcc_mul_2_over_3;
//
//	//Inverse Park
//	float v_alpha = v_d2 * cos_tetha - v_q2 * sin_tetha;
//	float v_beta = v_q2 * cos_tetha + v_d2 * sin_tetha;
//
//	float v_alpha2 = v_d * cos_tetha - v_q * sin_tetha;
//	float v_beta2 = v_q * cos_tetha + v_d * sin_tetha;

	//TODO dead time compensation

	//SVM
//	uint32_t duty1, duty2, duty3, svm_sector;
//	bldc_svm(v_alpha, v_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);
//
//	drv8301_set_pwm(duty1, duty2, duty3);
	//DRV8301_PWM_UPDATE_EVENT;	//TODO veryfi if required

	//Update angle
	//observer_update(v_alpha2, v_beta2, i_alpha, i_beta, &tetha);
	//pll_run(tetha, BLDC_DT, &m_pll_phase, &m_pll_speed);

	//tetha = tetha * (180.0f / (float) M_PI);
	/*
	tetha = tetha * RAD_TO_DEG;
	if(m_pll_speed<-10){
		tetha +=180;
		if(tetha>360){
			tetha-=360;
		}
	}*/

	///////////////////////////////////////////////////////////////////////////////

//
//const float PSIMH = MOTOR_LAMBDA;
//
//
//
//volatile float L = MOTOR_L * 3.0f / 2.0f;
//volatile float R = MOTOR_R * 3.0f / 2.0f;
//
//volatile float ed = v_d - R * i_d + w1 * L * i_q;
//volatile float eq = v_q - R * i_q - w1 * L * i_d;
//volatile float alpha = ALPHA0 + 2 * LAMBDA * fabsf(w1);
//
//if (w1 >= 0.0f) {
//	w1 += BLDC_DT * alpha * ((eq - LAMBDA * 1.0f * ed) / PSIMH - w1);
//} else {
//	w1 += BLDC_DT * alpha * ((eq - LAMBDA * -1.0f * ed) / PSIMH - w1);
//}
//theta_est += BLDC_DT * w1;
//while (theta_est > (float)M_PI){
//	theta_est -= 2.0f * (float)M_PI;
//}
//while (theta_est < -(float)M_PI){
//	theta_est += 2.0f * (float)M_PI;
//}
//
//pll_run(theta_est, BLDC_DT, &m_pll_phase, &m_pll_speed);
//tetha = theta_est * RAD_TO_DEG;

///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//	static float i_term = 0.0;
//	static float prev_error = 0.0;
//	float p_term;
//	float d_term;
//	float m_speed_pid_set_rpm;
//	if (tick_get_time_ms() < 10 * 1000) {
//		m_speed_pid_set_rpm = 1000.0f;
//	} else {
//		m_speed_pid_set_rpm = -1000.0f;
//	}
//
//	const float rpm = m_pll_speed;
//	float error = m_speed_pid_set_rpm - rpm;
//
////		// Too low RPM set. Reset state and return.
////		if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
////			i_term = 0.0;
////			prev_error = error;
////			return;
////		}
//
//	// Speed PID parameters
//
//	static uint32_t cnt = 0;
//	cnt++;
//	if (cnt == 25000 / 100) {
//		cnt = 0;
//		xx1 = tetha;
//		xx2 = i_q_ref;
//		xx3 = w1;
//		//xx4+=m_pll_speed * BLDC_DT* RAD_TO_DEG;
//	}
//
//#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))
//
//	// Compute parameters
//	p_term = error * MCCONF_S_PID_KP * (1.0 / 20.0);
//	i_term += error * MCCONF_S_PID_KI * (1.0 / 20.0);
//	d_term = (error - prev_error) * MCCONF_S_PID_KD * (1.0 / 20.0);
//
//	// Filter D
//	static float d_filter = 0.0;
//	UTILS_LP_FAST(d_filter, d_term, MCCONF_S_PID_KD_FILTER);
//	d_term = d_filter;
//
//	// I-term wind-up protection
//	utils_truncate_number(&i_term, -1.0, 1.0);
//
//	// Store previous error
//	prev_error = error;
//
//	// Calculate output
//	float output = p_term + i_term + d_term;
//	utils_truncate_number(&output, -1.0, 1.0);
//
//	i_q_ref = output * 4.0f;

	//Startup

//	static uint32_t start_cnt=0;
//	static uint32_t stepup_steps = 0;
//	start_cnt++;
//	if(start_cnt==DRV8301_PWM_3F_SWITCHING_FREQ_HZ/2){
//		start_cnt=0;
//		if(stepup_steps<25){		//5A
//			stepup_steps++;
//
//			i_q_ref+=0.2;
//		}
//	}


//	//PID speed
//	float speed_ref = 500.0f;		//TODO run with period 1000hz
//
//	//PID P error
//	float speed_err = speed_ref - m_pll_speed;
//
//
//	//PID I error
//	speed_err_acc += speed_err_acc * dt * BLDC_SPEED_PID_KI;
//
//	//PID I limit
//	if (speed_err_acc > BLDC_SPEED_PID_I_LIMIT) {
//		speed_err_acc = BLDC_SPEED_PID_I_LIMIT;
//	} else if (speed_err_acc < -BLDC_PID_I_LIMIT) {
//		speed_err_acc = -BLDC_PID_I_LIMIT;
//	}

	//PID out limit
	//float pid_speed_out = speed_err * BLDC_SPEED_PID_KP;// + speed_err_acc;

	//i_q_ref = pid_speed_out;
	//if(i_q_ref<0){
	//	i_q_ref=0;
	//}

	//Additional
//	g_i_d=i_d;
//	g_i_q=i_q;
//	g_v_d=v_d;
//	g_v_q=v_q;


//	if(tick_get_time_ms()>1000*70){
//		i_d_ref = 8;
//	}else if(tick_get_time_ms()>1000*60){
//		i_d_ref = 7;
//	}else if(tick_get_time_ms()>1000*50){
//		i_d_ref = 6;
//	}else if(tick_get_time_ms()>1000*40){
//		i_d_ref = 5;
//	}else if(tick_get_time_ms()>1000*30){
//		i_d_ref = 4;
//	}else if(tick_get_time_ms()>1000*20){
//		i_d_ref = 3;
//	}else if(tick_get_time_ms()>1000*10){
//		i_d_ref = 2;
//	}else{
//		i_d_ref = 1;
//	}

	///////////////////////////////////////////////////////////////////////////////////////////////////


	/*

	 float encoder_angle = TIM3->CNT % (4095 / 7);
	 encoder_angle = encoder_angle / (4095.0f / 7.0f) * 360.0f;

	 tetha = tetha * (180.0f / (float) M_PI);
	 */
	/*
	 static uint32_t cnt2 = 0;
	 static uint32_t cnt3 = 0;
	 //1
	 frame_bemf_voltage.a[cnt2] = tetha;

	 //2

	 frame_bemf_voltage.b[cnt2] = m_pll_phase* (180.0f / (float) M_PI);

	 //3
	 frame_bemf_voltage.c[cnt2] = m_pll_speed;

	 if (cnt3 == 4) {
	 cnt3 = 0;
	 cnt2++;
	 if (cnt2 == 5) {
	 cnt2 = 0;
	 frame_send(FRAME_TYPE_BEMF_VOLTAGE, (uint8_t *) &frame_bemf_voltage);
	 }
	 }
	 cnt3++;
	 */
	//tetha = encoder_angle;
	/*
	 tetha += offset;
	 if (tetha > 360) {
	 tetha -= 360;
	 }
	 */

	/* dynamicpid
	 //PID P error
	 float i_d_err = i_d_ref - i_d;
	 float i_q_err = i_q_ref - i_q;

	 float pid_p_d = i_d_err * BLDC_PID_KP;
	 float pid_p_q = i_q_err * BLDC_PID_KP;

	 float i_max;
	 //PID I limit
	 if(pid_p_d>0){
	 i_max = BLDC_PID_OUT_LIMIT - pid_p_d;
	 }else{
	 i_max = BLDC_PID_OUT_LIMIT + pid_p_d;
	 }

	 //PID I error
	 i_d_err_acc += i_d_err * dt * ki_idq;
	 i_q_err_acc += i_q_err * dt * ki_idq;

	 //PID I limit
	 if (pid_i_d > i_max) {
	 pid_i_d = i_max;
	 } else if (pid_i_d < -i_max) {
	 pid_i_d = -i_max;
	 }

	 if (pid_i_q > i_max) {
	 pid_i_q = i_max;
	 } else if (pid_i_q < -i_max) {
	 pid_i_q = -i_max;
	 }

	 //PID out
	 float v_d = pid_p_d + pid_i_d;
	 float v_q = pid_p_q + pid_i_q;
	 */
}

static void bldc_send_data(void){
	static uint32_t i=0;
	static uint8_t k=0;
	static FrameDisplayChannelsData8 frame;

	frame.ch1[i]=k++;
	frame.ch2[i]=k++;
	frame.ch3[i]=k++;
	frame.ch4[i]=k++;
	frame.ch5[i]=k++;
	frame.ch6[i]=k++;
	frame.ch7[i]=k++;
	frame.ch8[i]=k++;

	i++;
	if(i==FRAME_MAX_DISPLAY_CHANNELS_8){
		i=0;
		frame.packet_cnt++;

		//Slave->PC
		frame_uart_send(FRAME_TYPE_DISPLAY_CHANNELS_DATA_8, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
	}
}

void bldc_adc_irq_hanlder(void){
	//FOC Cycles Min, Max, AVR: 30, 37, 30.266

	//V LDO calculate
	//FOC Cycles Min, Max, AVR: 80, 102, 93.430
	//v_ldo_v = ADC_VREF_V * ((float) (*VREFINT_CAL)) / ((float) ADC_INJ_VREF_INT);

	//FOC Cycles Min, Max, AVR: 79, 95, 89.135
	//const float adc_vref_mul_vrefint_cal2 =  ADC_VREF_V * ((float) (*VREFINT_CAL));
	//v_ldo_v = adc_vref_mul_vrefint_cal2 / ((float) ADC_INJ_VREF_INT);

	//FOC Cycles Min, Max, AVR: 60, 91, 79.026 Next
	v_ldo_v = adc_vref_mul_vrefint_cal / ((float) ADC_INJ_VREF_INT);


	//TODO calculate 1/100, split calculation for severals loops
	//Calculate NTC temperature
	//FOC Cycles Min, Max, AVR: 203, 238, 221.814 Next
	float tmp;
	tmp = (ADC_NTC_R2_OHM * ((float) ADC_INJ_NTC)) / (ADC_MAX_VALUE - ((float) ADC_INJ_NTC));
	ntc_temperature_c = ADC_NTC_B_25_100_K / fast_log(tmp / ADC_NTC_R_INF) - ADC_KELVIN_OFFSET;

	//TODO calculate 1/100, split calculation for severals loops
	//Calculate uP temperature
	//FOC CFOC Cycles Min, Max, AVR: 274, 306, 292.696
	//tmp = (((float) ADC_INJ_TEMP_SENS) - (float) *ADC_TEMP30_CAL_ADDR) * (110.0f - 30.0f);
	//up_temperature_c = tmp / (float) (*ADC_TEMP110_CAL_ADDR - *ADC_TEMP30_CAL_ADDR) + 30.0f;

	//FOC Cycles Min, Max, AVR: 246, 280, 265.649 next
	tmp = (((float) ADC_INJ_TEMP_SENS) - (float) *ADC_TEMP30_CAL_ADDR) * (110.0f - 30.0f);
	up_temperature_c = tmp * one_over_adc_temp_call  + 30.0f;

	//Voltage calculation
	//FOC Cycles Min, Max, AVR: 285, 309, 303.299
	//v_vcc_v = ((float) ADC_INJ_VCC) * ADC_V_GAIN;
	//v_vcc_v *= v_ldo_v / ADC_MAX_VALUE;

	//FOC Cycles Min, Max, AVR: 275, 305, 293.599 next - not optimal
	float v_vcc_v_tmp = ((float) ADC_INJ_VCC) * v_ldo_v * (ADC_V_GAIN / ADC_MAX_VALUE);
	//LPF
	//v_vcc_v = v_vcc_v * 0.9f + v_vcc_v_tmp * 0.1f;
	v_vcc_v = v_vcc_v_tmp;
	//FOC Cycles Min, Max, AVR: 271, 290, 289.049
	//v_vcc_v = ((float) ADC_INJ_VCC) * 0.0013542013542014f * v_ldo_v;

	bldc_active_state_cb();
}

static void bldc_state_calibrate_finish(void){
	if (!drv8301_get_i_calibration_status()) {
		bldc_state_stop();
		bldc_set_active_state(BLDC_STATE_STOP);
	}
}

static void bldc_state_do_nothing(void){
	//TODO remove
	static bool start_foc = false;

	if(start_foc == false){
		start_foc = true;
		bldc_enable_all_pwm_output();
		DRV8301_PWM_UPDATE_EVENT;
		bldc_set_active_state(BLDC_STATE_FOC);
		//bldc_measure_r_init();
		//bldc_measure_l_init();
	}


}

void bldc_set_active_state(BldcStateMachine state){
	bldc_active_state = state;
	bldc_active_state_cb = state_dictionary[state].state_cb;
}
