#include "bldc.h"

//Calibration
#define BEMF_V_CALIBRATION_WAIT_TIME_MS			10

#define BLDC_R_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_R_SAMPLES					1024*8
#define BLDC_MEASURE_R_THETA_DEG				150.0f
#define BLDC_MEASURE_R_CURRENT_A				5.0f

#define BLDC_L_MEASUREMENT_START_TIME_MS		500
#define BLDC_MEASURE_L_WAIT_CYCLES				64
#define BLDC_MEASURE_L_DUTY						0.4
#define BLDC_MEASURE_L_SAMPLES					1024*BLDC_MEASURE_L_WAIT_CYCLES

static volatile float v1_bemf_h = 0;
static volatile float v2_bemf_h = 0;
static volatile float v3_bemf_h = 0;
static volatile float v_bemf_h_ldo = 0;
static volatile float v_bemf_h_vcc = 0;

static volatile float v1_bemf_offset = 0;
static volatile float v2_bemf_offset = 0;
static volatile float v3_bemf_offset = 0;
static volatile float v_bemf_offset_ldo = 0;
static volatile float v_bemf_offset_vcc = 0;

static volatile BldcStateMachine state_machine = BLDC_STATE_CALIBRATE_I;

static volatile float p1_i_offset = 0;
static volatile float p3_i_offset = 0;
static volatile float i_offset_ldo = 0;

//MOTO
#define MOTOR_R									0.088f
#define MOTOR_L									9.27e-6f
#define MOTOR_PID_TIME_CONSTANT					0.0005f

#define MOTOR_KA								(MOTOR_L/MOTOR_PID_TIME_CONSTANT)
#define MOTOR_KB								(MOTOR_R/MOTOR_L)
#define SQRT3_BY_2								0.86602540378f

//FOC
#define BLDC_DQ_LPF_CUTOFF_FREQ					2000.0f	//1000
#define BLDC_DT									(1.0f/(float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)
#define BLDC_VDQ_MAX_LIMIT						(SQRT3_BY_2 * 2.0f / 3.0f)

//#define BLDC_PID_KP 							MOTOR_KA					//0.2
//#define BLDC_PID_KI								MOTOR_KB * MOTOR_KA		//100

static volatile float kkp = MOTOR_KA;
static volatile float kki = MOTOR_KB * MOTOR_KA;

#define BLDC_PID_KP 							kkp
#define BLDC_PID_KI								kki
#define BLDC_PID_I_LIMIT						BLDC_VDQ_MAX_LIMIT
#define BLDC_PID_OUT_LIMIT 						BLDC_VDQ_MAX_LIMIT

#define ONE_BY_SQRT3							0.57735026919f
#define TWO_BY_SQRT3							(2.0f * 0.57735026919f)

float tetha = 0;
float i_d_ref = 0;
float i_q_ref = 0.5;		//2
float i_d_err_acc = 0;
float i_q_err_acc = 0;
float dt = BLDC_DT;
float i_d_lpf = 0;
float i_q_lpf = 0;

#define BLDC_DQ_LPF_ALPHA						BLDC_DQ_LPF_CUTOFF_FREQ/(BLDC_DQ_LPF_CUTOFF_FREQ+((float)DRV8301_PWM_3F_SWITCHING_FREQ_HZ)/(2.0f*M_PI))

float dq_lpf_alpha = BLDC_DQ_LPF_ALPHA;

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

void bldc_svm(float alpha, float beta, uint32_t PWMHalfPeriod, uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
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

static void bldc_calibrate_v(void) {
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
				v_bemf_offset_ldo += adc_get_v_ldo_v();
				v_bemf_offset_vcc += adc_get_v_vcc_v();

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

				state_machine = BLDC_STATE_CALIBRATE_FINISH;
			}
		} else {
			if (p_offset_cnt < ADC_V_OFFSET_COUNTER_MAX) {
				v1_bemf_h += ADC_INJ_P1_BEMF;
				v2_bemf_h += ADC_INJ_P2_BEMF;
				v3_bemf_h += ADC_INJ_P3_BEMF;
				v_bemf_h_ldo += adc_get_v_ldo_v();
				v_bemf_h_vcc += adc_get_v_vcc_v();

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

static void bldc_calibrate_i(void) {
	//Calibrate
	static uint32_t p_offset_cnt = 0;

	if (p_offset_cnt < ADC_I_OFFSET_COUNTER_MAX) {
		p1_i_offset += ADC_INJ_P1_I;
		p3_i_offset += ADC_INJ_P3_I;
		i_offset_ldo += adc_get_v_ldo_v();
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
		state_machine = BLDC_STATE_CALIBRATE_V;
	}
}

void bldc_stop(void) {
	//TODO optimize and verify
	drv8301_set_pwm(0, 0, 0);

	bldc_enable_all_pwm_output();
	DRV8301_PWM_UPDATE_EVENT;
}

void bldc_start(void) {
	state_machine = BLDC_STATE_FOC;
}

void bldc_measure_r_init(void) {
	//TODO check if stopped

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

	//Stop motor
	bldc_stop();
}

void bldc_measure_r(void) {
	//Get LDO voltage
	float v_ldo_v = adc_get_v_ldo_v();

	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//I3
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//VCC
	float vcc = adc_get_v_vcc_v();

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
	i_d_lpf = dq_lpf_alpha * i_d + (1.0f - dq_lpf_alpha) * i_d_lpf;
	i_q_lpf = dq_lpf_alpha * i_q + (1.0f - dq_lpf_alpha) * i_q_lpf;

	i_d = i_d_lpf;
	i_q = i_q_lpf;

	//PID P error
	float i_d_err = i_d_ref - i_d;
	float i_q_err = i_q_ref - i_q;

	//PID I error
	i_d_err_acc += i_d_err * dt * BLDC_PID_KI;
	i_q_err_acc += i_q_err * dt * BLDC_PID_KI;

	//PID I limit
	if (i_d_err_acc > BLDC_PID_I_LIMIT * vcc) {
		i_d_err_acc = BLDC_PID_I_LIMIT * vcc;
	} else if (i_d_err_acc < -BLDC_PID_I_LIMIT * vcc) {
		i_d_err_acc = -BLDC_PID_I_LIMIT * vcc;
	}

	if (i_q_err_acc > BLDC_PID_I_LIMIT * vcc) {
		i_q_err_acc = BLDC_PID_I_LIMIT * vcc;
	} else if (i_q_err_acc < -BLDC_PID_I_LIMIT * vcc) {
		i_q_err_acc = -BLDC_PID_I_LIMIT * vcc;
	}

	//PID out limit
	float v_d = i_d_err * BLDC_PID_KP + i_d_err_acc;
	float v_q = i_q_err * BLDC_PID_KP + i_q_err_acc;

	//TODO dynamic I limit

	//Maximum limitation
	float v_dq_mag = sqrtf(v_d * v_d + v_q * v_q);
	float v_dq_max = BLDC_VDQ_MAX_LIMIT * vcc;

	if (v_dq_mag > v_dq_max) {
		float dq_scale = v_dq_max / v_dq_mag;
		v_d *= dq_scale;
		v_q *= dq_scale;
	}

	float v_d2 = v_d / (vcc * 2.0f / 3.0f);
	float v_q2 = v_q / (vcc * 2.0f / 3.0f);

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

			bldc_stop();

			state_machine = BLDC_STATE_DO_NOTHING;
		}
	}
}

void bldc_measure_l_init(void) {
	//TODO check if stopped

	measure_l_i1_avr_1 = 0;
	measure_l_i3_avr_1 = 0;
	measure_l_vcc_avr_1 = 0;

	measure_l_i1_avr_2 = 0;
	measure_l_i3_avr_2 = 0;
	measure_l_vcc_avr_2 = 0;

	measure_l_cnt = 0;
	meaure_l_start_time = tick_get_time_ms();

	//Stop motor
	bldc_stop();
}

void bldc_measure_l(void) {
	//Get LDO voltage
	float v_ldo_v = adc_get_v_ldo_v();

	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//I3
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//VCC
	float vcc = adc_get_v_vcc_v();

	float duty = BLDC_MEASURE_L_DUTY;

	if (tick_get_time_ms() - meaure_l_start_time > BLDC_L_MEASUREMENT_START_TIME_MS) {

		if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 0) {
			//drv8301_set_pwm(0, duty * DRV8301_PWM_3F_PWM_MAX,0);
			drv8301_set_pwm(duty * DRV8301_PWM_3F_PWM_MAX, 0, duty * DRV8301_PWM_3F_PWM_MAX);
			DRV8301_PWM_UPDATE_EVENT;
		} else if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 2) {
			measure_l_i1_avr_1 += -p1_i;
			measure_l_i3_avr_1 += -p3_i;
			measure_l_vcc_avr_1 += vcc;

			drv8301_set_pwm(0, 0, 0);
			DRV8301_PWM_UPDATE_EVENT;
		} else if (measure_l_cnt % BLDC_MEASURE_L_WAIT_CYCLES == 3) {
			measure_l_i1_avr_2 += -p1_i;
			measure_l_i3_avr_2 += -p3_i;
			measure_l_vcc_avr_2 += vcc;

		}

		measure_l_cnt++;

		float duty_time = duty * BLDC_DT;

		if (measure_l_cnt == BLDC_MEASURE_L_SAMPLES) {
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
			float l = -(i1 * v2 + (-i2 - i1) * v1) * duty_time / (2.0f * i1 * i1) * 2.0f / 3.0f * 1e6;

			printf("Measured L AVR L[uH]: %f\n", l);

			bldc_stop();

			state_machine = BLDC_STATE_DO_NOTHING;
		}
	}
}

float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-20f; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + ((float) M_PI / 4.0f);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * (float) M_PI / 4.0f);
	}

	if (y < 0) {
		return (-angle);
	} else {
		return (angle);
	}
}

#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)
#define SQ(x)				((x) * (x))

float m_gamma_now = 5e8;
static float x1 = 0;
static float x2 = 0;

int utils_truncate_number_abs(float *number, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}

void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt, volatile float *phase) {
	float L = MOTOR_L * 3.0f / 2.0f;
	float lambda = 0.000609f;
	float R = MOTOR_R* 3.0f / 2.0f;


		const float L_ia = L * i_alpha;
		const float L_ib = L * i_beta;
		const float R_ia = R * i_alpha;
		const float R_ib = R * i_beta;
		const float lambda_2 = SQ(lambda);
		const float gamma_half = m_gamma_now * 0.5;

		// Original
	//	float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
	//	float x1_dot = -R_ia + v_alpha + gamma_half * (*x1 - L_ia) * err;
	//	float x2_dot = -R_ib + v_beta + gamma_half * (*x2 - L_ib) * err;
	//	*x1 += x1_dot * dt;
	//	*x2 += x2_dot * dt;

		// Iterative with some trial and error
		const int iterations = 6;
		const float dt_iteration = dt / (float)iterations;
		for (int i = 0;i < iterations;i++) {
			float err = lambda_2 - (SQ(x1 - L_ia) + SQ(x2 - L_ib));
			float gamma_tmp = gamma_half;
			if (utils_truncate_number_abs(&err, lambda_2 * 0.2)) {
				gamma_tmp *= 10.0;
			}
			float x1_dot = -R_ia + v_alpha + gamma_tmp * (x1 - L_ia) * err;
			float x2_dot = -R_ib + v_beta + gamma_tmp * (x2 - L_ib) * err;

			x1 += x1_dot * dt_iteration;
			x2 += x2_dot * dt_iteration;
		}

		// Same as above, but without iterations.
	//	float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
	//	float gamma_tmp = gamma_half;
	//	if (utils_truncate_number_abs(&err, lambda_2 * 0.2)) {
	//		gamma_tmp *= 10.0;
	//	}
	//	float x1_dot = -R_ia + v_alpha + gamma_tmp * (*x1 - L_ia) * err;
	//	float x2_dot = -R_ib + v_beta + gamma_tmp * (*x2 - L_ib) * err;
	//	*x1 += x1_dot * dt;
	//	*x2 += x2_dot * dt;

		UTILS_NAN_ZERO(x1);
		UTILS_NAN_ZERO(x2);

		*phase = utils_fast_atan2(x2 - L_ib, x1 - L_ia);


//	float L_ia = L * i_alpha;
//	float L_ib = L * i_beta;
//	float R_ia = R * i_alpha;
//	float R_ib = R * i_beta;
//
//	// Original
//	float err = lambda * lambda - ((x1 - L_ia) * (x1 - L_ia) + (x2 - L_ib) * (x2 - L_ib));
//	float x1_dot = -R_ia + v_alpha + m_gamma_now * 0.5f * (x1 - L_ia) * err;
//	float x2_dot = -R_ib + v_beta + m_gamma_now * 0.5f * (x2 - L_ib) * err;
//	x1 += x1_dot * dt;
//	x2 += x2_dot * dt;
//
//	UTILS_NAN_ZERO(x1);
//	UTILS_NAN_ZERO(x2);
//
//	*phase = utils_fast_atan2(x2 - L_ib, x1 - L_ia);
}

volatile float m_pll_phase = 0;
volatile float m_pll_speed = 0;

volatile float foc_pll_kp = 2000.0;
volatile float foc_pll_ki = 40000.0;


void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) {
		*angle += 2.0 * M_PI;
	}

	while (*angle > M_PI) {
		*angle -= 2.0 * M_PI;
	}
}

static void pll_run(float phase, float dt, volatile float *phase_var, volatile float *speed_var) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*) phase_var);
	*speed_var += foc_pll_ki * delta_theta * dt;
}

static volatile float g_i_abs=0;
static volatile float g_i_d=0;
static volatile float g_i_q=0;
static volatile float g_v_d=0;
static volatile float g_v_q=0;

void bldc_foc(void) {
	//Get LDO voltage
	float v_ldo_v = adc_get_v_ldo_v();

	//I1
	float p1_i = -(((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//I3
	float p3_i = -(((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;

	//VCC
	float vcc = adc_get_v_vcc_v();

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
	i_d_lpf = dq_lpf_alpha * i_d + (1.0f - dq_lpf_alpha) * i_d_lpf;
	i_q_lpf = dq_lpf_alpha * i_q + (1.0f - dq_lpf_alpha) * i_q_lpf;

	i_d = i_d_lpf;
	i_q = i_q_lpf;

	//PID P error
	float i_d_err = i_d_ref - i_d;
	float i_q_err = i_q_ref - i_q;

	//PID I error
	i_d_err_acc += i_d_err * dt * BLDC_PID_KI;
	i_q_err_acc += i_q_err * dt * BLDC_PID_KI;

	//PID I limit
	if (i_d_err_acc > BLDC_PID_I_LIMIT * vcc) {
		i_d_err_acc = BLDC_PID_I_LIMIT * vcc;
	} else if (i_d_err_acc < -BLDC_PID_I_LIMIT * vcc) {
		i_d_err_acc = -BLDC_PID_I_LIMIT * vcc;
	}

	if (i_q_err_acc > BLDC_PID_I_LIMIT * vcc) {
		i_q_err_acc = BLDC_PID_I_LIMIT * vcc;
	} else if (i_q_err_acc < -BLDC_PID_I_LIMIT * vcc) {
		i_q_err_acc = -BLDC_PID_I_LIMIT * vcc;
	}

	//PID out limit
	float v_d = i_d_err * BLDC_PID_KP + i_d_err_acc;
	float v_q = i_q_err * BLDC_PID_KP + i_q_err_acc;

	//Inject pulse
	/*
	static uint32_t pulse_cnt = 0;
	pulse_cnt++;

	if(pulse_cnt< 5){
		v_d += BLDC_VDQ_MAX_LIMIT / 8.0f;
	}

	if(pulse_cnt>DRV8301_PWM_3F_SWITCHING_FREQ_HZ / 250){ //TODO enum 250hz
		pulse_cnt=0;
	}
	*/
	//TODO dynamic I limit

	//Maximum limitation
	float v_dq_mag = sqrtf(v_d * v_d + v_q * v_q);
	float v_dq_max = BLDC_VDQ_MAX_LIMIT * vcc;

	if (v_dq_mag > v_dq_max) {
		float dq_scale = v_dq_max / v_dq_mag;
		v_d *= dq_scale;
		v_q *= dq_scale;
	}

	float v_d2 = v_d / (vcc * 2.0f / 3.0f);
	float v_q2 = v_q / (vcc * 2.0f / 3.0f);

	//Inverse Park
	float v_alpha = v_d2 * cos_tetha - v_q2 * sin_tetha;
	float v_beta = v_q2 * cos_tetha + v_d2 * sin_tetha;

	float v_alpha2 = v_d * cos_tetha - v_q * sin_tetha;
	float v_beta2 = v_q * cos_tetha + v_d * sin_tetha;

	//TODO dead time compensation

	//SVM
	uint32_t duty1, duty2, duty3, svm_sector;
	bldc_svm(v_alpha, v_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty3, &duty2, &svm_sector);

	drv8301_set_pwm(duty1, duty2, duty3);
	//DRV8301_PWM_UPDATE_EVENT;

	//Update angle
	observer_update(v_alpha2, v_beta2, i_alpha, i_beta, dt, &tetha);
	pll_run(tetha, dt, &m_pll_phase, &m_pll_speed);

	tetha = tetha * (180.0f / (float) M_PI);


	//Additional
	g_i_d=i_d;
	g_i_q=i_q;
	g_v_d=v_d;
	g_v_q=v_q;

/*
	if(tick_get_time_ms()>1000*70){
		i_d_ref = 8;
	}else if(tick_get_time_ms()>1000*60){
		i_d_ref = 7;
	}else if(tick_get_time_ms()>1000*50){
		i_d_ref = 6;
	}else if(tick_get_time_ms()>1000*40){
		i_d_ref = 5;
	}else if(tick_get_time_ms()>1000*30){
		i_d_ref = 4;
	}else if(tick_get_time_ms()>1000*20){
		i_d_ref = 3;
	}else if(tick_get_time_ms()>1000*10){
		i_d_ref = 2;
	}else{
		i_d_ref = 1;
	}
*/
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

void bldc_task(void) {
	switch (state_machine) {
	case BLDC_STATE_CALIBRATE_I:
		bldc_calibrate_i();
		break;

	case BLDC_STATE_CALIBRATE_V:
		bldc_calibrate_v();
		break;

	case BLDC_STATE_CALIBRATE_FINISH:
		if (!drv8301_get_i_calibration_status()) {
			bldc_stop();
			state_machine = BLDC_STATE_STOP;
		}
		break;

	case BLDC_STATE_STOP:
		//state_machine = BLDC_STATE_DO_NOTHING;
		state_machine = BLDC_STATE_FOC;

		//state_machine = BLDC_STATE_MEASURE_R;
		//bldc_measure_r_init();

		//state_machine = BLDC_STATE_MEASURE_L;
		//bldc_measure_l_init();
		break;

	case BLDC_STATE_FOC:
		//TODO enable
		bldc_foc();

		//TODO remove
		//bldc_send_data();
		break;

	case BLDC_STATE_MEASURE_R:
		bldc_measure_r();
		break;

	case BLDC_STATE_MEASURE_L:
		bldc_measure_l();
		break;

	case BLDC_STATE_DO_NOTHING:
		break;
		/*
		 case BLDC_STATE_INIT_ROTOR_POSITION:
		 //bldc_inject();
		 break;

		 case BLDC_STATE_INJECT_ROTATION_LOOP:
		 //bldc_inject_rotation_loop();
		 //state_machine = BLDC_STATE_FOC;
		 break;

		 case BLDC_STATE_SILENCE_BEFORE_INJECT:
		 //TODO ostatni impuls przed wstrzykiwaiem jest o polowe krotszy
		 //Todo pierwszy impuls po injection jest dluzszy
		 //state_machine = BLDC_STATE_INIT_ROTOR_POSITION;
		 break;

		 case BLDC_STATE_BEMF_ROTATION_LOOP:
		 //bldc_bemf_loop();
		 break;
		 */

	default:
		debug_error(BLDC_STATE_ERROR);
		break;
	}
}



//
//
//static float p1_i_offset = 0;
//static float p3_i_offset = 0;
//
//float dq_lpf_alpha = BLDC_DQ_LPF_ALPHA;
//

//
//float dt = BLDC_DT;
//float max_duty = SQRT3_BY_2;
//

//
//static volatile uint32_t rump_phase_offset = 3;	//todo make it as #define
//static volatile BldcInjectPhase phase_cnt = BLDC_INJECT_IAP;	//0
//static volatile BldcStateMachine state_machine = BLDC_STATE_CALIBRATE_I;
//
//static uint32_t rump_phase = 0;
////static FrameEncoderAngle frame_encoder_angle;
//
//#include <math.h>
//#include "arm_math.h"

//

//float i_d_lpf = 0;
//float i_q_lpf = 0;
//

//
///////////////////////////////////////////////////////////////////
//
//
///*
//static void bldc_measure_v_i(void) {
//	v_ldo_v = adc_get_v_ldo_v();
//
//	//I1
//	p1_i = (((float) ADC_INJ_P1_I) - p1_i_offset) / ADC_I_GAIN;
//	p1_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;
//
//	//I3
//	p3_i = (((float) ADC_INJ_P3_I) - p3_i_offset) / ADC_I_GAIN;
//	p3_i *= v_ldo_v / ADC_MAX_VALUE / ADC_I_R_OHM;
//
//	//I2
//	p2_i = -p1_i - p3_i;
//
//	//VCC
//	vcc = (float) ADC_INJ_VCC * ADC_V_GAIN;
//	vcc *= v_ldo_v / ADC_MAX_VALUE;
//
//	//V1
//	p1_bemf = ((float) ADC_INJ_P1_BEMF - ADC_MAX_VALUE / 2.0f) * ADC_V_GAIN;
//	p1_bemf *= v_ldo_v / ADC_MAX_VALUE;
//
//	//V2
//	p2_bemf = ((float) ADC_INJ_P2_BEMF - ADC_MAX_VALUE / 2.0f) * ADC_V_GAIN;
//	p2_bemf *= v_ldo_v / ADC_MAX_VALUE;
//
//	//V3
//	p3_bemf = ((float) ADC_INJ_P3_BEMF - ADC_MAX_VALUE / 2.0f) * ADC_V_GAIN;
//	p3_bemf *= v_ldo_v / ADC_MAX_VALUE;
//}
//*/
//void bldc_set_injected_phase(BldcInjectPhase phase) {
//	switch (phase) {
//	case BLDC_INJECT_IAP:
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//		break;
//
//	case BLDC_INJECT_IAN:
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//
//	case BLDC_INJECT_IBP:
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//		break;
//
//	case BLDC_INJECT_IBN:
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//
//	case BLDC_INJECT_ICP:
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//
//	case BLDC_INJECT_ICN:
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//		break;
//
//	case BLDC_INJECT_NONE:
//		bldc_phase_stop();
//		break;
//
//	default:
//		debug_error_handler(ROTOR_INJECTED_PHASE_ERROR);
//		break;
//	}
//}
//
//static float bldc_find_max(uint16_t *data) {
//	uint32_t i;
//	uint16_t max = 0;
//	for (i = 16; i < ADC_DMA_LENGTH; i++) {
//		if (data[i] > max) {
//			max = data[i];
//		}
//	}
//	return (float) max;
//}
//
//static float bldc_find_min(uint16_t *data) {
//	uint32_t i;
//	uint16_t min = 0xFFFF;
//	for (i = 16; i < ADC_DMA_LENGTH; i++) {
//		if (data[i] < min) {
//			min = data[i];
//		}
//	}
//	return (float) min;
//}
//
//static void bldc_inject(void) {
//	ADC_StopInjectedConversion(ADC2);
//	ADC_StopInjectedConversion(ADC4);
//
//	//DMA enable
//	DMA_Cmd(DMA2_Channel1, ENABLE);
//	DMA_Cmd(DMA2_Channel2, ENABLE);
//
//	//Set next phase
//	bldc_set_injected_phase(phase_cnt);
//	drv8301_set_pwm(DRV8301_PWM_3F_PWM_MAX / 4, DRV8301_PWM_3F_PWM_MAX / 4, DRV8301_PWM_3F_PWM_MAX / 4);
//
//	//Update PWM & phase event
//	TIM_SetCounter(TIM1, 0);
//	TIM_GenerateEvent(TIM1, TIM_EventSource_Update | TIM_EventSource_COM);
//
//	//LED_BLUE_ON;  todo ENABLE
//
//	//Start ADC
//	ADC_StartConversion(ADC2);
//	ADC_StartConversion(ADC4);
//}
//
//float angle_modulo(float angle) {
//	if (angle < 0) {
//		angle += 360.0f;
//	}
//
//	if (angle > 360.0f) {
//		angle -= 360.0f;
//	}
//
//	return angle;
//}
//
//static volatile uint32_t step = 0;
//
//static bool bldc_get_init_rotor_position(void) {
//	const float i_offset[6] = { 1055.159939f, 1129.334505f, 1171.007183f, 1047.553928f, 1030.116211f, 1125.892122f };
//	static float i_max[6];	//AP, AN, BP, BN, CP, CN
//
//	//Stop ADC
//	ADC_StopConversion(ADC2);
//	ADC_StopConversion(ADC4);
//
//	bldc_set_injected_phase(BLDC_INJECT_NONE);
//	drv8301_set_pwm(0, 0, 0);
//	DRV8301_PWM_UPDATE_EVENT;
//
//	//Find max min I
//	uint16_t *adc_i1 = adc_get_dma_adc2_buffer();
//	uint16_t *adc_i3 = adc_get_dma_adc4_buffer();
//
//	//Measure injected current
//	switch (phase_cnt) {
//	case BLDC_INJECT_IAP:
//		i_max[0] = -i_offset[0] + fabs(bldc_find_max(adc_i1) - p1_i_offset);
//		break;
//	case BLDC_INJECT_IAN:
//		i_max[1] = -i_offset[1] + fabs(bldc_find_min(adc_i1) - p1_i_offset);
//		break;
//	case BLDC_INJECT_IBP:
//		i_max[2] = -i_offset[2] + fabs(bldc_find_min(adc_i1) - p1_i_offset + bldc_find_min(adc_i3) - p3_i_offset);
//		break;
//	case BLDC_INJECT_IBN:
//		i_max[3] = -i_offset[3] + fabs(bldc_find_max(adc_i1) - p1_i_offset + bldc_find_max(adc_i3) - p3_i_offset);
//		break;
//	case BLDC_INJECT_ICP:
//		i_max[4] = -i_offset[4] + fabs(bldc_find_max(adc_i3) - p3_i_offset);
//		break;
//	case BLDC_INJECT_ICN:
//		i_max[5] = -i_offset[5] + fabs(bldc_find_min(adc_i3) - p3_i_offset);
//		break;
//	default:
//		debug_error_handler(BLDC_PHASE_CNT_ERROR);
//		break;
//	}
//
//	//DMA reinit
//	adc_dma_reinit();
//
//	//Next phase
//	phase_cnt++;
//	if (phase_cnt == 6) {
//
//		//DC calculate
//		float i_p_dc = (i_max[0] + i_max[2] + i_max[4]) / 3.0f;
//		float i_n_dc = (i_max[1] + i_max[3] + i_max[5]) / 3.0f;
//
//		i_max[0] -= i_p_dc;
//		i_max[1] -= i_n_dc;
//		i_max[2] -= i_p_dc;
//		i_max[3] -= i_n_dc;
//		i_max[4] -= i_p_dc;
//		i_max[5] -= i_n_dc;
//
//		//Find local max
//		uint32_t i;
//
//		uint32_t max_i = 0;
//		for (i = 0; i < 6; i++) {
//			if (i_max[i] > i_max[max_i]) {
//				max_i = i;
//			}
//		}
//
//		float y = 0;
//		float z = 0;
//		float fi = 0;
//
//		switch (max_i) {
//		case 0:
//			case 1:
//			if (i_max[0] > i_max[1]) {
//				rump_phase = 3; //3/3
//
//				fi = (float) M_PI;
//				z = i_max[4] - i_max[5];		//cp - cn
//				y = i_max[2] - i_max[3];		//bp - bn
//			} else {
//				rump_phase = 0; 			//0
//
//				fi = 0;
//				z = i_max[5] - i_max[4];		//cn - cp
//				y = i_max[3] - i_max[2];		//bn - bp
//			}
//			break;
//		case 2:
//			case 3:
//			if (i_max[2] > i_max[3]) {
//				rump_phase = 5; 			//5/3
//
//				fi = 5.0f * (float) M_PI / 3.0f;
//				z = i_max[0] - i_max[1];		//ap - an
//				y = i_max[4] - i_max[5];		//cp - cn
//			} else {
//				rump_phase = 2; 			//2/3
//
//				fi = 2.0f * (float) M_PI / 3.0f;
//				z = i_max[1] - i_max[0];		//an - ap
//				y = i_max[5] - i_max[4];		//cn - cp
//			}
//
//			break;
//		case 4:
//			case 5:
//			if (i_max[4] > i_max[5]) {
//				rump_phase = 1; 			//1/3
//
//				fi = (float) M_PI / 3.0f;
//				z = i_max[2] - i_max[3];		//bp - bn
//				y = i_max[0] - i_max[1];		//ap - an
//			} else {
//				rump_phase = 4; 			//4/3
//
//				fi = 4.0f * (float) M_PI / 3.0f;
//				z = i_max[3] - i_max[2];		//bn - bp
//				y = i_max[1] - i_max[0];		//an - ap
//			}
//			break;
//		}
//
//		rump_phase += rump_phase_offset;
//		rump_phase %= 6;
//
//		volatile float angle_est = (fi - (y - z) / (z + y) / 2) * 180.0f / (float) M_PI;
//
//		if (angle_est < 0) {
//			angle_est += 360.0f;
//		}
//
//		if (angle_est > 360.0f) {
//			angle_est -= 360.0f;
//		}
//
//		TIM3->CNT = 4095.0f / 7.0f * angle_est / 360.0f;
//		//Frame data
//		static uint32_t cnt2 = 0;
//		//frame_encoder_angle.angle[cnt2] = TIM3->CNT;
//		//frame_encoder_angle.estimation[cnt2] = angle_est;
//
//		//frame_encoder_angle.angle[cnt2] = rump_phase * 100;
//		//frame_encoder_angle.estimation[cnt2] = step * 100;
//
//		cnt2++;
//		if (cnt2 == 25) {
//			cnt2 = 0;
//			//frame_send(FRAME_TYPE_ENCODER_ANGLE, (uint8_t *) &frame_encoder_angle);
//		}
//
//		//bldc_phase_step(rump_phase);
//		//drv8301_set_pwm(DRV8301_PWM_3F_PWM_MAX / 8, DRV8301_PWM_3F_PWM_MAX / 8, DRV8301_PWM_3F_PWM_MAX / 8);
//		//DRV8301_PWM_UPDATE_EVENT;
//
//		state_machine = BLDC_STATE_INJECT_ROTATION_LOOP;
//		phase_cnt = BLDC_INJECT_IAP;
//
//		ADC_StartInjectedConversion(ADC2);
//		ADC_StartInjectedConversion(ADC4);
//
//		return false;
//	}
//	return true;
//}
//
//void bldc_inject_next(void) {
//	//LED_BLUE_OFF;
//	DRV8301_PWM_DISABLE;
//
//	if (bldc_get_init_rotor_position()) {
//		bldc_inject();
//	}
//
//	DRV8301_PWM_ENABLE;
//}
//
//void bldc_phase_step(uint32_t step) {
//	//-----------------------------------------------------------
//	//|----------| Step1 | Step2 | Step3 | Step4 | Step5 | Step6 |
//	//-----------------------------------------------------------
//	//|Channel1  |   1   |   0   |   0   |   0   |   0   |   1   |
//	//-----------------------------------------------------------
//	//|Channel1N |   0   |   0   |   1   |   1   |   0   |   0   |
//	//-----------------------------------------------------------
//	//|Channel2  |   0   |   0   |   0   |   1   |   1   |   0   |
//	//-----------------------------------------------------------
//	//|Channel2N |   1   |   1   |   0   |   0   |   0   |   0   |
//	//-----------------------------------------------------------
//	//|Channel3  |   0   |   1   |   1   |   0   |   0   |   0   |
//	//-----------------------------------------------------------
//	//|Channel3N |   0   |   0   |   0   |   0   |   1   |   1   |
//	//-----------------------------------------------------------
//
//	switch (step) {
//	case 0:
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//	case 1:
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//
//		break;
//	case 2:
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//	case 3:
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//		break;
//	case 4:
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//
//		break;
//	case 5:
//		//0
//		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//		//+
//		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//		//-
//		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//		break;
//	default:
//		debug_error_handler(BLDC_PHASE_CNT_ERROR);
//		break;
//	}
//}
//
//void bldc_phase_stop(void) {
//	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//
//	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
//
//	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
//}
//
//volatile float int_treshold = 0.05f;
//volatile static float integral = 0;
//
////volatile FrameBemfVoltage frame_bemf_voltage;
//volatile uint32_t bldc_step = 0;
//
//static void bldc_inject_rotation_loop(void) {
//
//	if (tick_get_time_ms() < 500) {
//		bldc_phase_step(rump_phase);
//		drv8301_set_pwm(DRV8301_PWM_3F_PWM_MAX / 4, DRV8301_PWM_3F_PWM_MAX / 4, DRV8301_PWM_3F_PWM_MAX / 4);
//		DRV8301_PWM_UPDATE_EVENT;
//		bldc_step = rump_phase;
//	} else {
//		state_machine = BLDC_STATE_BEMF_ROTATION_LOOP;
//	}
//
//	//
//	static uint32_t cnt = 0;
//	if (cnt == 25) {
//		cnt = 0;
//		state_machine = BLDC_STATE_SILENCE_BEFORE_INJECT;
//		drv8301_set_pwm(0, 0, 0);
//		DRV8301_PWM_UPDATE_EVENT;
//	} else {
//		cnt++;
//	}
//}
//
//void bldc_bemf_loop(void) {
//	/*
//	//Observe BEMF
//	bldc_measure_v_i();
//
//	float bemf_a = p1_bemf;
//	float bemf_b = p2_bemf;
//	float bemf_c = p3_bemf;
//
//	float vcc_half = (bemf_a + bemf_b + bemf_c) / 3.0f;
//
//	//Compensate BEMF DC
//	bemf_a -= vcc_half;
//	bemf_b -= vcc_half;
//	bemf_c -= vcc_half;
//
//
////	 static uint32_t cnt2 = 0;
////	 static uint32_t cnt3 = 0;
////	 frame_bemf_voltage.a[cnt2] = integral;
////	 frame_bemf_voltage.b[cnt2] = bldc_step;
////	 frame_bemf_voltage.c[cnt2] = 0;
////
////	 if (cnt3 == 10) {
////	 cnt3 = 0;
////	 cnt2++;
////	 if (cnt2 == 10) {
////	 cnt2 = 0;
////	 frame_send(FRAME_TYPE_BEMF_VOLTAGE, (uint8_t *) &frame_bemf_voltage);
////	 }
////	 }
////	 cnt3++;
//
//	static uint32_t dead_counter = 0;
//	static uint32_t last_step = 0;
//
//	if (last_step != bldc_step) {
//		last_step = bldc_step;
//		dead_counter = 0;
//	}
//
//	dead_counter++;
//	if (dead_counter > 1) {
//		dead_counter--;
//		switch (bldc_step) {
//		case 0:
//			integral += bemf_c * +1;
//			break;
//		case 1:
//			integral += bemf_b * -1;
//			break;
//		case 2:
//			integral += bemf_a * +1;
//			break;
//		case 3:
//			integral += bemf_c * -1;
//			break;
//		case 4:
//			integral += bemf_b * +1;
//			break;
//		case 5:
//			integral += bemf_a * -1;
//			break;
//		}
//
//		if (integral < 0) {
//			integral = 0;
//		}
//	} else {
//		integral = 0;
//	}
//
//	if (integral / 100.0f > (float) int_treshold / 2.0f) {
//		bldc_step++;
//		bldc_step %= 6;
//		//LED_BLUE_TOGGLE;  todo ENABLE
//	}
//
//	bldc_phase_step(bldc_step);
//	drv8301_set_pwm(DRV8301_PWM_3F_PWM_MAX / 2, DRV8301_PWM_3F_PWM_MAX / 2, DRV8301_PWM_3F_PWM_MAX / 2);
//	DRV8301_PWM_UPDATE_EVENT;
//	*/
//}
//
//
//int utils_truncate_number_abs(float *number, float max) {
//	int did_trunc = 0;
//
//	if (*number > max) {
//		*number = max;
//		did_trunc = 1;
//	} else if (*number < -max) {
//		*number = -max;
//		did_trunc = 1;
//	}
//
//	return did_trunc;
//}
//#define UTILS_IS_NAN(x)		((x) != (x))
//#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)
//

//
//bool utils_saturate_vector_2d(float *x, float *y, float max) {
//	bool retval = false;
//	float mag = sqrtf(*x * *x + *y * *y);
//	max = fabsf(max);
//
//	if (mag < 1e-10) {
//		mag = 1e-10;
//	}
//
//	if (mag > max) {
//		const float f = max / mag;
//		*x *= f;
//		*y *= f;
//		retval = true;
//	}
//
//	return retval;
//}
//
//float kp_idq = 0.1f;
//float ki_idq = 0.5f;
//
//
//
//
//
//void utils_norm_angle_rad(float *angle) {
//	while (*angle < -M_PI) {
//		*angle += 2.0 * M_PI;
//	}
//
//	while (*angle > M_PI) {
//		*angle -= 2.0 * M_PI;
//	}
//}
///*
//volatile float m_pll_phase = 0;
//volatile float m_pll_speed = 0;
//
//volatile float foc_pll_kp = 2000.0;
//volatile float foc_pll_ki = 40000.0;
//
//static void pll_run(float phase, float dt, volatile float *phase_var, volatile float *speed_var) {
//	UTILS_NAN_ZERO(*phase_var);
//	float delta_theta = phase - *phase_var;
//	utils_norm_angle_rad(&delta_theta);
//	UTILS_NAN_ZERO(*speed_var);
//	*phase_var += (*speed_var + foc_pll_kp * delta_theta) * dt;
//	utils_norm_angle_rad((float*) phase_var);
//	*speed_var += foc_pll_ki * delta_theta * dt;
//}
//**/
///*
//volatile float offset = 90;
//
//volatile float v_a = 0;
//volatile float v_b = 0;
//volatile float v_c = 0;
//float i1_lpf = 0;
//float i2_lpf = 0;
//float i3_lpf = 0;
//*/
//void bldc_foc(void) {
//	/*
//	bldc_measure_v_i();
//
//	//Clarke Transform
//	float i_alpha = p1_i;
//	float i_beta = ONE_BY_SQRT3 * p1_i + TWO_BY_SQRT3 * p2_i;
//
//	//Park Transform
//	float sin_tetha;
//	float cos_tetha;
//	arm_sin_cos_f32(tetha, &sin_tetha, &cos_tetha);
//
//	float i_d = i_alpha * cos_tetha + i_beta * sin_tetha;
//	float i_q = i_beta * cos_tetha - i_alpha * sin_tetha;
//
//	//DC link current calculation
//	float i_dc_link = sqrtf(i_d * i_d + i_q * i_q);
//
//	//Current LPF
//	i_d_lpf = i_d_lpf * lpf_coef + (1.0f - lpf_coef) * i_d;
//	i_q_lpf = i_q_lpf * lpf_coef + (1.0f - lpf_coef) * i_q;
//
//	i_d = i_d_lpf;
//	i_q = i_q_lpf;
//
//	//Id and Iq reference
//	float i_d_ref = 0.5f;	//todo add torque control loop
//	float i_q_ref = 0;		//todo add weaking
//
//	//PID P error
//	float i_d_err = i_d_ref - i_d;
//	float i_q_err = i_q_ref - i_q;
//
//	//PID output Vd, Vq
//	float v_d = kp_idq * i_d_err + i_d_err_acc;
//	float v_q = kp_idq * i_q_err + i_q_err_acc;
//
//	//Limit the PID output
//	//float max_duty = 0.5f; //<=SQRT3_BY_2
//	float max_duty = SQRT3_BY_2;
//	utils_saturate_vector_2d(&v_d, &v_q, max_duty);
//
//	//PID I error
//	i_d_err_acc += i_d_err * dt * ki_idq;
//	i_q_err_acc += i_q_err * dt * ki_idq;
//
//	//Limit I component
//	utils_truncate_number_abs(&i_d_err_acc, max_duty);
//	utils_truncate_number_abs(&i_q_err_acc, max_duty);
//
//	//Inverse Park
//	float v_alpha = v_d * cos_tetha - v_q * sin_tetha;
//	float v_beta = v_q * cos_tetha + v_d * sin_tetha;
//
//	//TODO dead time compensation
//
//	//SVM
//	uint32_t duty1, duty2, duty3, svm_sector;
//	svm(-v_alpha, -v_beta, DRV8301_PWM_3F_PWM_MAX, &duty1, &duty2, &duty3, &svm_sector);
//	drv8301_set_pwm(duty1, duty2, duty3);
//
//	//Todo, minimise
//	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
//	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
//
//	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
//	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
//	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
//
//	DRV8301_PWM_UPDATE_EVENT;
//
//	//Calculate Vd, Vq voltage
//	v_alpha = v_alpha * (2.0f / 3.0f) * vcc;
//	v_beta = v_beta * (2.0f / 3.0f) * vcc;
//
//	observer_update(v_alpha, v_beta, i_alpha, i_beta, dt, &tetha);
//	pll_run(tetha, dt, &m_pll_phase, &m_pll_speed);
//
//
////	 static uint32_t cnt5=0;
////	 cnt5++;
////	 if(cnt5==20000){
////	 offset+=1;
////	 cnt5=0;
////	 }
//
//
//	i1_lpf = i1_lpf * lpf_coef + (1.0f - lpf_coef) * p1_i;
//	i2_lpf = i2_lpf * lpf_coef + (1.0f - lpf_coef) * p2_i;
//	i3_lpf = i3_lpf * lpf_coef + (1.0f - lpf_coef) * p3_i;
//
//	float encoder_angle = TIM3->CNT % (4095 / 7);
//	encoder_angle = encoder_angle / (4095.0f / 7.0f) * 360.0f;
//
//	tetha = tetha * (180.0f / (float) M_PI);
//
//
////	 static uint32_t cnt2 = 0;
////	 static uint32_t cnt3 = 0;
////	 //1
////	 frame_bemf_voltage.a[cnt2] = tetha;
////
////	 //2
////
////	 frame_bemf_voltage.b[cnt2] = m_pll_phase* (180.0f / (float) M_PI);
////
////	 //3
////	 frame_bemf_voltage.c[cnt2] = m_pll_speed;
////
////	 if (cnt3 == 4) {
////	 cnt3 = 0;
////	 cnt2++;
////	 if (cnt2 == 5) {
////	 cnt2 = 0;
////	 frame_send(FRAME_TYPE_BEMF_VOLTAGE, (uint8_t *) &frame_bemf_voltage);
////	 }
////	 }
////	 cnt3++;
//
//	//tetha = encoder_angle;
//	tetha += offset;
//	if (tetha > 360) {
//		tetha -= 360;
//	}
//*/
//}

