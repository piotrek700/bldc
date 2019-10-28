#include "drv8301.h"
#include "drv8301_tran.h"

static bool init_status = false;
static volatile bool calibration_status = false;
static volatile uint16_t status_reg1 = 0;
static volatile uint16_t status_reg2 = 0;

static void drv8301_check_fault(uint16_t * value) {
	if ((*value & DRV8301_READ_FAULT)) {
		//TODO enable
		//debug_error(DRV8301_READ_FRAME_FAULT);
	}
}

//1
void drv8301_status_register1_cb(uint8_t *rx) {
	uint16_t *stat_reg1 = (uint16_t *) rx;
	*stat_reg1 = SWAP_UINT16(*stat_reg1);

	drv8301_check_fault(stat_reg1);

	status_reg1 = (*stat_reg1) & DRV8301_SR1_FAULT_MASK;
}

//2
void drv8301_status_register2_cb(uint8_t *rx) {
	uint16_t *stat_reg2 = (uint16_t *) rx;
	*stat_reg2 = SWAP_UINT16(*stat_reg2);

	drv8301_check_fault(stat_reg2);

	status_reg2 = (*stat_reg2) & DRV8301_SR2_FAULT_MASK;
}

//3
void drv8301_control_register1_cb(uint8_t *rx) {
	uint16_t *ctrl_reg1 = (uint16_t *) rx;

	drv8301_check_fault(ctrl_reg1);
}

//4
void drv8301_control_register2_cb(uint8_t *rx) {
	uint16_t *ctrl_reg2 = (uint16_t *) rx;

	drv8301_check_fault(ctrl_reg2);
}

//6
void drv8301_calib_enable_cb(uint8_t *rx) {
	uint16_t *ctrl_reg2 = (uint16_t *) rx;

	drv8301_check_fault(ctrl_reg2);

	calibration_status = true;
}

//7
void drv8301_calib_disable_cb(uint8_t *rx) {
	uint16_t *ctrl_reg2 = (uint16_t *) rx;

	drv8301_check_fault(ctrl_reg2);

	calibration_status = false;
}

static void drv8301_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	//P1H 	PA08
	//P1L	PA11
	//P2H	PA09
	//P2L 	PA12
	//P3H	PA10
	//P3L	PB15
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_4);

	//EN_GATE C14
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//nFAULT PA15 BREAK
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_9);
}

static void drv8301_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0);
}

static void drv8301_fault_irq(void) {
	drv8301_read_status();

	//TODO read registers from drv8301
	//send massgae about staus to pc
	//frame_send_message_string(MSG_ERROR, (uint8_t *)"DRF8301 FAULT fault");
	//LED_RED_ON;
}

void TIM1_BRK_TIM15_IRQHandler(void) {
	rybos_task_start_marker(MARKER_IRQ_DRV_FAULT);

	if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
		drv8301_fault_irq();
	}

	rybos_task_stop_marker(MARKER_IRQ_DRV_FAULT);
}

static void drv8301_timer_init(void) {
	RCC_TIMCLKConfig(RCC_TIM1CLK_PLLCLK);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_DeInit(TIM1);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;		//No mether REF 565
	TIM_TimeBaseStructure.TIM_Period = DRV8301_PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_DeadTime = DRV8301_PWM_3F_DEAD_TIME_CYCLES;
	//TODO TIM_Break_Enable
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	//Enable - resume PWM in next period, disable - wit for software resume
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	//Block timer brake register, clear by reset
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	//
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	//If break appear, disable output and set values TIM_OCIdleState_Reset
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

	TIM_CCPreloadControl(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	//Very important to update at proper sequence, must be set after timer enable
	TIM1->RCR = 1;
}

void drv8301_set_pwm(uint16_t ch1, uint16_t ch2, uint16_t ch3) {
	if (ch1 > DRV8301_PWM_3F_PWM_MAX) {
		TIM1->CCR1 = DRV8301_PWM_3F_PWM_MAX;
	}

	if (ch2 > DRV8301_PWM_3F_PWM_MAX) {
		TIM1->CCR2 = DRV8301_PWM_3F_PWM_MAX;
	}

	if (ch3 > DRV8301_PWM_3F_PWM_MAX) {
		TIM1->CCR3 = DRV8301_PWM_3F_PWM_MAX;
	}

	TIM1->CCR1 = ch1;
	TIM1->CCR2 = ch2;
	TIM1->CCR3 = ch3;
}

static void drv8301_register_init(void) {
	spi_add_transaction((SpiTransactionRecord *) &clear_trans);
	spi_add_transaction((SpiTransactionRecord *) &write_ctrl_reg1);
	spi_add_transaction((SpiTransactionRecord *) &write_ctrl_reg2);
}

void drv8301_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	drv8301_gpio_init();
	DRV8301_EN_GATE_DISABLE;
	tick_delay_ms(2);

	drv8301_timer_init();
	drv8301_set_pwm(0, 0, 0);
	DRV8301_PWM_UPDATE_EVENT;

	DRV8301_EN_GATE_ENABLE;
	tick_delay_ms(10);

	drv8301_nvic_init();
	drv8301_register_init();

	//Calibrate current offset
	drv8301_i_calibration_enable();

	drv8301_test();

	init_status = true;
}

void drv8301_read_status(void) {
	spi_add_transaction((SpiTransactionRecord *) &read_stat_reg1);
	spi_add_transaction((SpiTransactionRecord *) &read_stat_reg2);
}

bool drv8301_get_i_calibration_status(void) {
	return calibration_status;
}

void drv8301_i_calibration_enable(void) {
	spi_add_transaction((SpiTransactionRecord *) &write_calib_enable);
}

void drv8301_i_calibration_disable(void) {
	spi_add_transaction((SpiTransactionRecord *) &write_calib_disable);
}

uint16_t drv8301_get_status_reg1(void) {
	return status_reg1;
}

uint16_t drv8301_get_status_reg2(void) {
	return status_reg2;
}

bool drv8301_get_init_status(void) {
	return init_status;
}

void drv8301_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}
