#include "adc.h"
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../Rybos/rybos.h"
#include "../Drv8301/drv8301.h"
#include "../Bldc/bldc.h"
#include "../utils.h"
#include "../Led/led.h"

static bool init_status = false;

static volatile uint32_t adc_calib_value_ch1 = 0;
static volatile uint32_t adc_calib_value_ch2 = 0;
static volatile uint32_t adc_calib_value_ch3 = 0;
static volatile uint32_t adc_calib_value_ch4 = 0;

static void adc_gpio_init(void) {
	//NTC		ADC1	CH2		PA1
	//P1_I		ADC2	CH1		PA4
	//VBAT		ADC3	CH1		PB1
	//P3_I		ADC4 	CH3		PB12
	//P1_BEMF	ADC2	CH12	PB2
	//P2_BEMF	ADC3	CH5		PB13
	//P3_BEMF	ADC4	CH4		PB14

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void adc_adc1_init(void) {
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

	ADC_DeInit(ADC1);

	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_TempSensorCmd(ADC1, ENABLE);
	ADC_VrefintCmd(ADC1, ENABLE);
	ADC_VbatCmd(ADC1, ENABLE);

	tick_delay_ms(2);

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);

	uint32_t timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((ADC_GetCalibrationStatus(ADC1) != RESET) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	adc_calib_value_ch1 = ADC_GetCalibrationValue(ADC1);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge; 	  //ADC_ExternalTrigInjecEventEdge_None;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 4;
	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_Channel_Ntc;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_Channel_TempSensor;
	ADC_InjectedInitStruct.ADC_InjecSequence3 = ADC_Channel_Vbat;
	ADC_InjectedInitStruct.ADC_InjecSequence4 = ADC_Channel_Vrefint;
	ADC_InjectedInit(ADC1, &ADC_InjectedInitStruct);

	//NTC		ADC1	CH2		PA1
	//TMP		ADC1	CH16
	//VBAT		ADC1	CH17
	//Vref		ADC1	CH18
	ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_Ntc, ADC_SampleTime_181Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_TempSensor, ADC_SampleTime_181Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_Vbat, ADC_SampleTime_181Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_Vrefint, ADC_SampleTime_181Cycles5);

	ADC_ITConfig(ADC1, ADC_IT_JEOS, ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY)) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	//ADC_StartInjectedConversion(ADC1);
}

static void adc_adc234_init(void) {
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div1);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);

	//ADC_DeInit(ADC2);	//done in adc_adc1_init()
	ADC_DeInit(ADC3);
	ADC_DeInit(ADC4);

	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);

	tick_delay_ms(2);

	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);

	ADC_StartCalibration(ADC2);
	ADC_StartCalibration(ADC3);
	ADC_StartCalibration(ADC4);

	uint32_t timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((ADC_GetCalibrationStatus(ADC2) != RESET) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((ADC_GetCalibrationStatus(ADC3) != RESET) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((ADC_GetCalibrationStatus(ADC4) != RESET) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	adc_calib_value_ch2 = ADC_GetCalibrationValue(ADC2);
	adc_calib_value_ch3 = ADC_GetCalibrationValue(ADC3);
	adc_calib_value_ch4 = ADC_GetCalibrationValue(ADC4);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;

	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_Init(ADC4, &ADC_InitStructure);

	//Injected channels
	ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0; 		//REF manual 330
	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2;

	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_Channel_P1_I;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_Channel_P1_Bemf;
	ADC_InjectedInit(ADC2, &ADC_InjectedInitStruct);

	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_Channel_Vcc;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_Channel_P2_Bemf;
	ADC_InjectedInit(ADC3, &ADC_InjectedInitStruct);

	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_Channel_P3_I;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_Channel_P3_Bemf;
	ADC_InjectedInit(ADC4, &ADC_InjectedInitStruct);

	//P1_I		ADC2	CH1		PA4
	//VBAT		ADC3	CH1		PB1
	//P3_I		ADC4 	CH3		PB12
	//P1_BEMF	ADC2	CH12	PB2
	//P2_BEMF	ADC3	CH5		PB13
	//P3_BEMF	ADC4	CH4		PB14

	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_P1_I, ADC_SAMPLING_CYCLES);
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_P1_Bemf, ADC_SAMPLING_CYCLES);
	ADC_InjectedChannelSampleTimeConfig(ADC3, ADC_Channel_Vcc, ADC_SAMPLING_CYCLES);
	ADC_InjectedChannelSampleTimeConfig(ADC3, ADC_Channel_P2_Bemf, ADC_SAMPLING_CYCLES);
	ADC_InjectedChannelSampleTimeConfig(ADC4, ADC_Channel_P3_I, ADC_SAMPLING_CYCLES);
	ADC_InjectedChannelSampleTimeConfig(ADC4, ADC_Channel_P3_Bemf, ADC_SAMPLING_CYCLES);

	ADC_ITConfig(ADC4, ADC_IT_JEOS, ENABLE);

	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY)) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY)) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

	timeout_cnt = ADC_TIEMOUT_CYCLES;
	while ((!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY)) && (--timeout_cnt != 0));

	if (timeout_cnt == 0) {
		debug_error(ADC_TIMEOUT_ERROR);
	}

}

static void adc_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(ADC1_2_IRQn, 3);

//	NVIC_InitStructure.NVIC_IRQChannel = ADC4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_Init(&NVIC_InitStructure);
//
//	NVIC_SetPriority(ADC4_IRQn, 2);
}

CCMRAM_FUCNTION void ADC1_2_IRQHandler(void) {
	rybos_task_start_marker(RYBOS_MARKER_IRQ_ADC_BLDC);

	//ADC1 clear pending IRQ bit
	ADC1->ISR = (uint32_t) 0x7FF;
	ADC2->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;
	ADC3->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;
	ADC4->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;

//	 //Check if ADC1 IRQ complete
//	if (ADC_GetITStatus(ADC1, ADC_IT_JEOS)) {
//		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOS);
//	} else {
//		debug_error_handler(ADC1_OVERRUN_ERROR);
//	}
//
//	//Check if ADC4 IRQ complete
//	if (ADC_GetITStatus(ADC4, ADC_IT_JEOS)) {
//		ADC_ClearITPendingBit(ADC4, ADC_IT_JEOS);
//	} else {
//		debug_error_handler(ADC234_OVERRUN_ERROR);
//	}
//
//	//ADC1 overrun protection
//	if (ADC_GetITStatus(ADC1, ADC_IT_OVR)) {
//		debug_error_handler(ADC1_OVERRUN_ERROR);
//	}
//
//	//ADC4 overrun protection
//	if (ADC_GetITStatus(ADC4, ADC_IT_OVR)) {
//		debug_error_handler(ADC234_OVERRUN_ERROR);
//	}

	//FOC
	bldc_adc_irq_hanlder();

	rybos_task_stop_marker(RYBOS_MARKER_IRQ_ADC_BLDC);
}

void adc_init(void) {
	adc_gpio_init();
	adc_nvic_init();
	adc_adc1_init();
	adc_adc234_init();

	ADC1->ISR = (uint32_t) 0x7FF;
	ADC2->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;
	ADC3->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;
	ADC4->ISR = (uint32_t) 0x7FF;	//ADC_IT_JEOS;

//	ADC_StartInjectedConversion(ADC1);
//	ADC_StartInjectedConversion(ADC2);
//	ADC_StartInjectedConversion(ADC3);
//	ADC_StartInjectedConversion(ADC4);

	ADC1->CR |= ADC_CR_JADSTART;
	ADC2->CR |= ADC_CR_JADSTART;
	ADC3->CR |= ADC_CR_JADSTART;
	ADC4->CR |= ADC_CR_JADSTART;

	adc_test();

	init_status = true;
}

void adc_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool adc_get_init_status(void) {
	return init_status;
}
