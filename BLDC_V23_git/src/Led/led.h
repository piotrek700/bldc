#ifndef LED_H_
#define LED_H_

#include "platform.h"

#define LED_RED_ON			GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define LED_RED_OFF 		GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define LED_RED_TOGGLE		GPIOB->ODR ^= GPIO_Pin_11
#define LED_RED_CHECK		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

//#define LED_BLUE_ON			GPIO_SetBits(GPIOB, GPIO_Pin_10)
//#define LED_BLUE_OFF 			GPIO_ResetBits(GPIOB, GPIO_Pin_10)
//#define LED_BLUE_TOGGLE		GPIOB->ODR ^= GPIO_Pin_10
//#define LED_BLUE_CHECK		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

#define LED_TEST_DELAY_MS	500

void led_init(void);

bool led_get_init_status(void);

#endif
