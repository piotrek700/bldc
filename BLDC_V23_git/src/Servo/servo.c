#include "servo.h"
#include <sdk/debug.h>
#include <sdk/uuid.h>

static bool init_status = false;

static float servo_angle[4] = { 0, 0, 0, 0 };
static float servo_angle_offset[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

void servo_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

static void servo_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_2);	//Servo 1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_2);	//Servo 2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);	//Servo 3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);	//Servo 4
}

static void servo_timer_init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_DeInit(TIM4);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = SERVO_CLOCK_PRESCALER - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SERVO_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = SERVO_PERIOD / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_Cmd(TIM4, ENABLE);
}

static void servo_offset_init(void) {
	switch (STM32_UUID[0]) {
	case 0x00400028:
		servo_angle_offset[0] = -4.0f;
		servo_angle_offset[1] = 2.0f;
		servo_angle_offset[2] = 0.0f;
		servo_angle_offset[3] = -2.0f;
		break;

	case 0x00350018:
		servo_angle_offset[0] = 7.0f;
		servo_angle_offset[1] = -1.0f;
		servo_angle_offset[2] = 11.0f;
		servo_angle_offset[3] = 5.0f;
		break;

	default:
		//debug_error(SERVO_OFFSET_NOT_DEFINED);
		break;
	}
}

void servo_init(void) {
	servo_gpio_init();
	servo_timer_init();
	servo_offset_init();

	servo_test();

	servo_set_all(0);

	init_status = true;
}

bool servo_get_init_status(void) {
	return init_status;
}

void servo_set_position_angle(ServoPosition servo_position, float angle) {
	//Save position
	servo_angle[servo_position] = angle;

	if (servo_angle[servo_position] < SERVO_MIN_ANGLE) {
		servo_angle[servo_position] = SERVO_MIN_ANGLE;
	}

	if (servo_angle[servo_position] > SERVO_MAX_ANGLE) {
		servo_angle[servo_position] = SERVO_MAX_ANGLE;
	}

	//Add offset
	angle += servo_angle_offset[servo_position];

	if (angle < SERVO_MIN_ANGLE) {
		angle = SERVO_MIN_ANGLE;
	}

	if (angle > SERVO_MAX_ANGLE) {
		angle = SERVO_MAX_ANGLE;
	}

	float poistion_f = -angle / 90.0f * (float) SERVO_PERIOD / 20.0f + 1.5f * (float) SERVO_PERIOD / 20.0f;

	switch (servo_position) {
	case SERVO_POSITION_1_LEFT:
		TIM4->CCR1 = (uint16_t) poistion_f;
		break;
	case SERVO_POSITION_2_TOP:
		TIM4->CCR2 = (uint16_t) poistion_f;
		break;
	case SERVO_POSITION_3_RIGHT:
		TIM4->CCR3 = (uint16_t) poistion_f;
		break;
	case SERVO_POSITION_4_BOTTOM:
		TIM4->CCR4 = (uint16_t) poistion_f;
		break;
	default:
		debug_error(SERVO_POSITION_NOT_DEFINED);
		break;
	}
}

float *servo_get_angle(void) {
	return servo_angle;
}

void servo_set_all(float angle) {
	servo_set_position_angle(SERVO_POSITION_1_LEFT, angle);
	servo_set_position_angle(SERVO_POSITION_2_TOP, angle);
	servo_set_position_angle(SERVO_POSITION_3_RIGHT, angle);
	servo_set_position_angle(SERVO_POSITION_4_BOTTOM, angle);
}
