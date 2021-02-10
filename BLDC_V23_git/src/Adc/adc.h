#ifndef ADC_H_
#define ADC_H_

#include "platform.h"
#include <sdk/utils.h>

#define ADC_SAMPLING_CYCLES 		ADC_SampleTime_19Cycles5

#define ADC_Channel_Ntc 			ADC_InjectedChannel_2

#define ADC_INJ_NTC					ADC1->JDR1
#define ADC_INJ_TEMP_SENS			ADC1->JDR2
#define ADC_INJ_VBAT				ADC1->JDR3
#define ADC_INJ_VREF_INT			ADC1->JDR4

#define ADC_Channel_P1_I 			ADC_InjectedChannel_1
#define ADC_Channel_Vcc 			ADC_InjectedChannel_1
#define ADC_Channel_P3_I 			ADC_InjectedChannel_3
#define ADC_Channel_P1_Bemf 		ADC_InjectedChannel_12						//TODO one channel has slow conversion, change during next design
#define ADC_Channel_P2_Bemf 		ADC_InjectedChannel_5
#define ADC_Channel_P3_Bemf 		ADC_InjectedChannel_4

#define ADC_INJ_P1_I				ADC2->JDR1
#define ADC_INJ_VCC					ADC3->JDR1
#define ADC_INJ_P3_I				ADC4->JDR1
#define ADC_INJ_P1_BEMF				ADC2->JDR2
#define ADC_INJ_P2_BEMF				ADC3->JDR2
#define ADC_INJ_P3_BEMF				ADC4->JDR2

#define ADC_VREF_V					3.3f
#define ADC_VREF_COMPENSATION		(3.3f/3.39f)								//Set to 1 if new chip revision will be used
#define ADC_MAX_VALUE				4095.0f

#define ADC_NTC_B_25_100_K			3455.0f
#define ADC_NTC_R0_OHM				10000.0f
#define ADC_NTC_R2_OHM				10000.0f
#define ADC_KELVIN_OFFSET			273.15f
#define ADC_NTC_T0_K				(25.0f+ADC_KELVIN_OFFSET)
#define ADC_NTC_R_INF				0.09275567347300997333f						//r_inf=ADC_NTC_R0_OHM*exp(-ADC_NTC_B_25_100_K/ADC_NTC_T0_K)

#define ADC_MIN_BAT_V				9.5f										//3.1f*3+margin
#define ADC_NTC_MAX_TEMP_C			75.0f

#define ADC_I_OFFSET_COUNTER_MAX	4096*2
#define ADC_V_OFFSET_COUNTER_MAX	4096

#define ADC_V_GAIN					((15.0f+3.3f)/3.3f)
#define ADC_I_GAIN					19.6078431372549f	//20.0f, corrected by 100k/(5k + 100R) 										//DRV OP amp gain
#define ADC_I_R_OHM					0.004f

#define ADC_TIEMOUT_CYCLES			500000

UNUSED_WARNING_DISABLE static volatile uint16_t *VREFINT_CAL = (uint16_t *) ((uint32_t) 0x1FFFF7BA);
UNUSED_WARNING_DISABLE static volatile uint16_t *ADC_TEMP110_CAL_ADDR = (uint16_t *) ((uint32_t) 0x1FFFF7C2);
UNUSED_WARNING_DISABLE static volatile uint16_t *ADC_TEMP30_CAL_ADDR = (uint16_t *) ((uint32_t) 0x1FFFF7B8);

void adc_init(void);

bool adc_get_init_status(void);

void adc_dma_reinit(void);

uint16_t * adc_get_dma_adc2_buffer(void);

uint16_t * adc_get_dma_adc4_buffer(void);

#endif
