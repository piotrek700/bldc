#ifndef DRV8301_H_
#define DRV8301_H_

#include "platform.h"
#include "drv8301_reg.h"
#include "../Tick/tick.h"

#define DRV8301_PWM_3F_SWITCHING_FREQ_HZ			20000		//TODO can be even 25000
#define DRV8301_PWM_PERIOD							(TICK_CPU_FREQUENCY_HZ / DRV8301_PWM_3F_SWITCHING_FREQ_HZ)
//#define DRV8301_PWM_3F_DEAD_TIME_CYCLES			25	//TODO must be 25,    200 -2.22222us		//max reg value 255
#define DRV8301_PWM_3F_DEAD_TIME_CYCLES				25
#define DRV8301_PWM_3F_PWM_MAX 						DRV8301_PWM_PERIOD
#define DRV8301_PWM_3F_PWM_MIN						0

#define	DRV8301_EN_GATE_ENABLE						GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define DRV8301_EN_GATE_DISABLE 					GPIO_ResetBits(GPIOC, GPIO_Pin_14)
#define DRV8301_EN_GATE_TOGGLE						GPIOC->ODR ^= GPIO_Pin_14
#define DRV8301_EN_GATE_CHECK						GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)

#define DRV8301_RESUME_AFTER_BREAK					TIM_CtrlPWMOutputs(TIM1, ENABLE)
#define DRV8301_PWM_UPDATE_EVENT					TIM_GenerateEvent(TIM1, TIM_EventSource_COM)

#define DRV8301_CR1_SETTINGS						( DRV8301_CR1_GATE_CURRENT_1P7A	  | DRV8301_CR1_GATE_RESET_NORMAL_MODE	| DRV8301_CR1_PWM_MODE_6PWM 	 | DRV8301_CR1_OCP_MODE_OC_LATCH_SHUT_DOWN	| DRV8301_CR1_OC_ADJ_SET_0P511 )
#define DRV8301_CR2_SETTINGS						( DRV8301_CR2_OCTW_MODE_OT_AND_OC | DRV8301_CR2_GAIN_20V 				| DRV8301_CR2_DC_CAL_CH1_CONNECT | DRV8301_CR2_DC_CAL_CH2_CONNECT			| DRV8301_CR2_OC_TOFF_CYCLE_By_CYCLE )
//TODO change to 20x
#define DRV8301_PWM_DISABLE							TIM_CtrlPWMOutputs(TIM1, DISABLE)
#define DRV8301_PWM_ENABLE							TIM_CtrlPWMOutputs(TIM1, ENABLE)

void drv8301_init(void);

bool drv8301_get_init_status(void);

void drv8301_test(void);

void drv8301_set_pwm(uint16_t ch1, uint16_t ch2, uint16_t ch3);

void drv8301_read_status(void);

uint16_t drv8301_get_status_reg1(void);

uint16_t drv8301_get_status_reg2(void);

bool drv8301_get_i_calibration_status(void);

void drv8301_i_calibration_enable(void);

void drv8301_i_calibration_disable(void);

void drv8301_status_register1_cb(uint8_t *rx);

void drv8301_status_register2_cb(uint8_t *rx);

void drv8301_control_register1_cb(uint8_t *rx);

void drv8301_control_register2_cb(uint8_t *rx);

void drv8301_calib_enable_cb(uint8_t *rx);

void drv8301_calib_disable_cb(uint8_t *rx);

#endif
