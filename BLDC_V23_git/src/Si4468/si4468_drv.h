#ifndef SI4468_DRV_H_
#define SI4468_DRV_H_

#include "Spi/spi.h"

#define SI4468_DRV_IRQ_CHECK			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define SI4468_DRV_CTS_CHECK			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

void si4468_drv_add_transaction_blocking(SpiTransactionRecord_t *record);

void si4468_drv_nvic_init(void);

void si4468_drv_init(void);

void si4468_drv_test(void);

bool si4468_drv_get_init_status(void);

#endif
