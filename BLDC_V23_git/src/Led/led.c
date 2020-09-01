#include "led.h"
#include "../Debug/debug.h"

static bool init_status = false;

void led_gpio_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = /*GPIO_Pin_10 |*/GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void led_init(void) {
	led_gpio_init();

	LED_RED_OFF;
	//LED_BLUE_OFF;

	led_test();

	init_status = true;
}

void led_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}

	//TODO Test
}

bool led_get_init_status(void) {
	return init_status;
}

