#include <devices/si4468.h>
#include <devices/si4468_tran.h>
#include <sdk/rybos.h>
#include <sdk/debug.h>
#include "si4468_drv.h"

static bool init_status = false;

void __attribute__((weak)) si4468_drv_nirq_cb(void) {
	debug_error(SI4468_NIRQ_NOT_IMPLEMENTED);
}

void __attribute__((weak)) si4468_drv_cts_cb(void) {
	debug_error(SI4468_CTS_NOT_IMPLEMENTED);
}

static void si4468_drv_gpio_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void si4468_drv_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(EXTI9_5_IRQn, 6);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(EXTI15_10_IRQn, 7);
}

static void si4468_drv_exti_init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
}

void EXTI9_5_IRQHandler(void) {
	rybos_task_start_marker(RYBOS_MARKER_IRQ_SI4468_NIRQ);

	//IRQ
	EXTI_ClearITPendingBit(EXTI_Line5);

	si4468_drv_nirq_cb();

	rybos_task_stop_marker(RYBOS_MARKER_IRQ_SI4468_NIRQ);
}

void EXTI15_10_IRQHandler(void) {
	rybos_task_start_marker(RYBOS_MARKER_IRQ_SI4468_CTS);

	//CTS
	EXTI_ClearITPendingBit(EXTI_Line10);

	si4468_drv_cts_cb();

	rybos_task_stop_marker(RYBOS_MARKER_IRQ_SI4468_CTS);
}

void si4468_drv_add_transaction_blocking(SpiTransactionRecord *record) {
	EXTI_ClearITPendingBit(EXTI_Line10);
	spi_add_transaction(record);
	while (EXTI_GetITStatus(EXTI_Line10) == 0);
	EXTI_ClearITPendingBit(EXTI_Line10);

	while (SI4468_DRV_CTS_CHECK == 0) {
		while (EXTI_GetITStatus(EXTI_Line10) == 0);
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

void si4468_drv_init(void) {
	if (!spi_get_init_status()) {
		spi_init();
	}

	si4468_drv_gpio_init();
	si4468_drv_exti_init();

	si4468_drv_test();

	init_status = true;
}

void si4468_drv_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool si4468_drv_get_init_status(void) {
	return init_status;
}
