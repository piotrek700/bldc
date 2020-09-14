#include "buzzer_timer.h"
#include <sdk/tick.h>
#include <sdk/debug.h>

static bool init_status = false;

static void buzzer_timer_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
}

static void buzzer_timer_timer_init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_DeInit(TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

void buzzer_timer_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

void buzzer_timer_init(void) {
	buzzer_timer_gpio_init();
	buzzer_timer_timer_init();

	buzzer_timer_test();

	init_status = true;
}

bool buzzer_timer_get_init_status(void) {
	return init_status;
}

void buzzer_timer_set_frequency(uint16_t freq) {
	if (freq == 0) {
		TIM2->CCR1 = 0;
	} else {
		uint16_t prescaler = TICK_CPU_FREQUENCY_HZ / freq / 0xFFFF + 1;
		uint16_t period = TICK_CPU_FREQUENCY_HZ / prescaler / freq;
		TIM2->PSC = prescaler - 1;
		TIM2->ARR = period;
		TIM2->CCR1 = period / 2;
		TIM2->EGR = TIM_PSCReloadMode_Immediate;

//		uint32_t period = TICK_CPU_FREQUENCY_HZ / freq;
//		TIM2->ARR = period;
//		TIM2->CCR1 = period / 2;
//		TIM2->EGR = TIM_PSCReloadMode_Immediate;
	}
}
