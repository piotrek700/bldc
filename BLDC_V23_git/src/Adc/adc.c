#include "adc.h"

static volatile uint16_t *VREFINT_CAL = (uint16_t *) ((uint32_t) 0x1FFFF7BA);
static volatile uint16_t *ADC_TEMP110_CAL_ADDR = (uint16_t *) ((uint32_t) 0x1FFFF7C2);
static volatile uint16_t *ADC_TEMP30_CAL_ADDR = (uint16_t *) ((uint32_t) 0x1FFFF7B8);

static bool init_status = false;
static bool bldc_enable = false;

static volatile uint32_t adc_calib_value_ch1 = 0;
static volatile uint32_t adc_calib_value_ch2 = 0;
static volatile uint32_t adc_calib_value_ch3 = 0;
static volatile uint32_t adc_calib_value_ch4 = 0;

static volatile float v_vcc_v = 0;
static volatile float v_ldo_v = 0;
static volatile float up_temperature_c = 0;
static volatile float ntc_temperature_c = 0;

static volatile uint16_t adc2_dma[ADC_DMA_LENGTH];
static volatile uint16_t adc4_dma[ADC_DMA_LENGTH];

static volatile uint32_t left_cycles=0;
static volatile float left_time =0;

static void adc_stop_bldc(void){
	//TODO implement
}

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

	tick_delay_ms(1);

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

	tick_delay_ms(1);

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
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
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

	//Regular channels
	//ADC_RegularChannelConfig(ADC2, ADC_Channel_P1_I, 1, ADC_SAMPLING_CYCLES);
	//ADC_RegularChannelConfig(ADC4, ADC_Channel_P3_I, 1, ADC_SAMPLING_CYCLES);

	//ADC_DMAConfig(ADC2, ADC_DMAMode_OneShot);
	//ADC_DMAConfig(ADC4, ADC_DMAMode_OneShot);

	//ADC_DMACmd(ADC2, ENABLE);
	//ADC_DMACmd(ADC4, ENABLE);

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

	//ADC_StartInjectedConversion(ADC2);
	//ADC_StartInjectedConversion(ADC3);
	//ADC_StartInjectedConversion(ADC4);
}

static void adc_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(ADC1_2_IRQn, 4);
/*
	NVIC_InitStructure.NVIC_IRQChannel = ADC4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(ADC4_IRQn, 3);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(DMA2_Channel2_IRQn, 1);
	*/
}



float adc_bldc_left_time(void){
	return left_time;
}

void ADC1_2_IRQHandler(void) {															//3%
	LED_RED_ON;									//TODO remove
	LED_RED_OFF;

	left_cycles = tick_get_clock_tick()-left_cycles;
	//asm volatile("" ::: "memory");
	//LED_RED_ON;
	rybos_task_start_marker(MARKER_IRQ_ADC_NTC);										//15%

	//ADC1 clear pending iqr bit 	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOS);
	ADC1->ISR = (uint32_t) 0x7FF;
	ADC2->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;
	ADC3->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;
	ADC4->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;


	//Check if ADC1 IRQ complete
	if (ADC_GetITStatus(ADC1, ADC_IT_JEOS)) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOS);
	}else{
		//debug_error_handler(ADC1_OVERRUN_ERROR);
	}

	//Check if ADC4 IRQ complete
	if (ADC_GetITStatus(ADC4, ADC_IT_JEOS)) {
		ADC_ClearITPendingBit(ADC4, ADC_IT_JEOS);
	}else{
		//debug_error_handler(ADC234_OVERRUN_ERROR);
	}

	//ADC1 overrun protection
	if (ADC_GetITStatus(ADC1, ADC_IT_OVR)) {
		//debug_error_handler(ADC1_OVERRUN_ERROR);
	}

	//ADC4 overrun protection
	if (ADC_GetITStatus(ADC4, ADC_IT_OVR)) {
		//debug_error_handler(ADC234_OVERRUN_ERROR);
	}

	//Check left time
	//left_time = 100.0f * (float) DRV8301_PWM_3F_SWITCHING_FREQ_HZ * (float) left_cycles / (float) TICK_CPU_FREQUENCY_HZ;
	left_time = 0.0277777777777778f * left_cycles;


	//V LDO calculate
	v_ldo_v = ADC_VREF_V * ((float) (*VREFINT_CAL)) / ((float) ADC_INJ_VREF_INT);		//21%

	//TODO calclate 1/100
	//Calculate NTC temperature
	float tmp;
	tmp = (ADC_NTC_R2_OHM * ((float) ADC_INJ_NTC)) / (ADC_MAX_VALUE - ((float) ADC_INJ_NTC));
	ntc_temperature_c = ADC_NTC_B_25_100_K / fast_log(tmp / ADC_NTC_R_INF) - ADC_KELVIN_OFFSET;

	//TODO calclate 2/100
	//Calculate uP temperature
	tmp = (((float) ADC_INJ_TEMP_SENS) - (float) *ADC_TEMP30_CAL_ADDR) * (110.0f - 30.0f);
	up_temperature_c = tmp / (float) (*ADC_TEMP110_CAL_ADDR - *ADC_TEMP30_CAL_ADDR) + 30.0f;

	//Voltage calculation
	//v_vcc_v = ((float) ADC_INJ_VCC) * ADC_V_GAIN;
	//v_vcc_v *= v_ldo_v / ADC_MAX_VALUE;
	v_vcc_v = ((float) ADC_INJ_VCC) * v_ldo_v * 0.0013542013542014f;

	if(bldc_enable){
		//TODO redesign that to use reference
		bldc_task();
	}

	rybos_task_stop_marker(MARKER_IRQ_ADC_NTC);		//15%
	//LED_RED_OFF;
	//asm volatile("" ::: "memory");
	left_cycles = tick_get_clock_tick();
}
/*
void ADC4_IRQHandler(void) {
	rybos_task_start_marker(MARKER_IRQ_ADC_BLDC);
	ADC_ClearITPendingBit(ADC4, ADC_IT_JEOS);

	if (ADC_GetITStatus(ADC4, ADC_IT_OVR)) {
		debug_error_handler(ADC234_OVERRUN_ERROR);
	}

	v_vcc_v = (float) ADC_INJ_VCC * ADC_V_GAIN;
	v_vcc_v *= v_ldo_v / ADC_MAX_VALUE;

	if(bldc_enable){
		bldc_task();
	}

	rybos_task_stop_marker(MARKER_IRQ_ADC_BLDC);
}
*/
uint16_t * adc_get_dma_adc2_buffer(void) {
	return (uint16_t *) adc2_dma;
}

uint16_t * adc_get_dma_adc4_buffer(void) {
	return (uint16_t *) adc4_dma;
}

static void adc_dma_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	DMA_DeInit(DMA2_Channel1); //ADC2 DMA_CH1
	DMA_DeInit(DMA2_Channel2); //ADC4 DMA_CH2

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_BufferSize = (uint16_t) ADC_DMA_LENGTH;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc2_dma;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC2->DR;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);

	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc4_dma;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC4->DR;
	DMA_Init(DMA2_Channel2, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Channel2, DMA_IT_TC, ENABLE);
}

void adc_dma_reinit(void) {
	//TODO replace function bz operation on register to speed up
	DMA_ClearFlag(DMA2_FLAG_GL1);
	DMA_ClearFlag(DMA2_FLAG_GL2);

	DMA_Cmd(DMA2_Channel1, DISABLE);	//ADC2
	DMA_Cmd(DMA2_Channel2, DISABLE);	//ADC4

	ADC_DMACmd(ADC2, DISABLE);
	ADC_DMACmd(ADC4, DISABLE);

	DMA2_Channel1->CNDTR = ADC_DMA_LENGTH;
	DMA2_Channel2->CNDTR = ADC_DMA_LENGTH;

	ADC_DMACmd(ADC2, ENABLE);
	ADC_DMACmd(ADC4, ENABLE);

	DMA_Cmd(DMA2_Channel1, ENABLE);
	DMA_Cmd(DMA2_Channel2, ENABLE);
}

void DMA2_Channel2_IRQHandler(void) {
	if (DMA_GetITStatus(DMA2_IT_TC2) != RESET) {
		DMA_ClearITPendingBit(DMA2_IT_TC2);

		//bldc_inject_next();
	}
}

void adc_init(void) {
	adc_gpio_init();
	adc_nvic_init();
	adc_dma_init();
	adc_adc1_init();
	adc_adc234_init();

	ADC1->ISR = (uint32_t) 0x7FF;
	ADC2->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;
	ADC3->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;
	ADC4->ISR = (uint32_t) 0x7FF;//ADC_IT_JEOS;

	ADC1->CR |= ADC_CR_JADSTART;
	ADC2->CR |= ADC_CR_JADSTART;
	ADC3->CR |= ADC_CR_JADSTART;
	ADC4->CR |= ADC_CR_JADSTART;

	//ADC_StartInjectedConversion(ADC1);
	//ADC_StartInjectedConversion(ADC2);
	//ADC_StartInjectedConversion(ADC3);
	//ADC_StartInjectedConversion(ADC4);

	adc_test();

	init_status = true;
}

float adc_get_v_ldo_v(void) {
	return v_ldo_v;
}

float adc_get_v_vcc_v(void) {
	return v_vcc_v;
}

float adc_get_ntc_temperature_c(void) {
	return ntc_temperature_c;
}

float adc_get_up_temperature_c(void) {
	return up_temperature_c;
}

void adc_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool adc_bldc_status(void) {
	return bldc_enable;
}

void adc_set_bldc_enable(bool status) {
	bldc_enable = status;

	if(bldc_enable==false){
		adc_stop_bldc();
	}
}

bool adc_get_init_status(void) {
	return init_status;
}
