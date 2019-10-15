#include "buzzer.h"
#include "buzzer_sounds.h"

static bool init_status = false;

static const BuzzerSoundStep *sound_ptr;
static uint32_t sound_length = 0;
static uint32_t sound_step = 1;

static const BuzzerDictionaryRow sound_dictionary[] = {
		{ buzzer_sound_start, 		sizeof(buzzer_sound_start) / 		sizeof(BuzzerSoundStep)}, //BUZZER_SOUND_START
		{ buzzer_sound_mario, 		sizeof(buzzer_sound_mario) / 		sizeof(BuzzerSoundStep)}, //BUZZER_SOUND_MARIO
		{ buzzer_sound_single_peak, sizeof(buzzer_sound_single_peak) /  sizeof(BuzzerSoundStep)}, //BUZZER_SOUND_SINGLE_PEAK
		{ buzzer_sound_double_peak, sizeof(buzzer_sound_double_peak) /  sizeof(BuzzerSoundStep)}, //BUZZER_SOUND_DOUBLE_PEAK
		{ buzzer_sound_triple_peak, sizeof(buzzer_sound_triple_peak) /  sizeof(BuzzerSoundStep)}, //BUZZER_SOUND_TRIPLE_PEAK
		{ buzzer_sound_turn_off, 	sizeof(buzzer_sound_turn_off) / 	sizeof(BuzzerSoundStep)}  //BUZZER_SOUND_TURN_OFF
};

void buzzer_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

static void buzzer_gpio_init(void) {
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

static void buzzer_timer_init(void) {
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

void buzzer_init(void) {
	buzzer_gpio_init();
	buzzer_timer_init();

	buzzer_test();

	init_status = true;
}

bool buzzer_get_init_status(void) {
	return init_status;
}

static void buzzer_set_frequency(uint32_t freq) {
	if (freq == 0) {
		TIM2->CCR1 = 0;
	} else {
		uint16_t prescaler = TICK_CPU_FREQUENCY_HZ / freq / 0xFFFF + 1;
		uint16_t period = TICK_CPU_FREQUENCY_HZ / prescaler / freq;
		TIM2->PSC = prescaler - 1;
		TIM2->ARR = period;
		TIM2->CCR1 = period / 2;
		TIM2->EGR = TIM_PSCReloadMode_Immediate;

		//uint32_t period = TICK_CPU_FREQUENCY_HZ / freq;
		//TIM2->ARR = period;
		//TIM2->CCR1 = period / 2;
		//TIM2->EGR = TIM_PSCReloadMode_Immediate;
	}
}

void buzzer_generate_sound(BuzzerSoundType sound_type) {
	sound_ptr = sound_dictionary[sound_type].sound_ptr;
	sound_length = sound_dictionary[sound_type].sound_length;
	sound_step = 0;
	rybos_task_enable(MARKER_TASK_BUZZER, true);
}

void buzzer_state_machine(void) {
	static uint32_t next_time_step = 0;

	if (sound_step <= sound_length) {
		if (sound_step == 0) {
			next_time_step = tick_get_time_ms() + sound_ptr[sound_step].generation_time_ms;
			buzzer_set_frequency(sound_ptr[sound_step].frequency_hz);
			sound_step++;
		} else {
			if (tick_get_time_ms() >= next_time_step) {
				if (sound_step == sound_length) {
					buzzer_set_frequency(0);
					rybos_task_enable(MARKER_TASK_BUZZER, false);
					return;
				}
				next_time_step += sound_ptr[sound_step].generation_time_ms;
				buzzer_set_frequency(sound_ptr[sound_step].frequency_hz);
				sound_step++;
			}
		}
	}
}

