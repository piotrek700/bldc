#ifndef SPI_H_
#define SPI_H_

#include "platform.h"

#define SPI_IMU_NSS_SET_HIGH	 			GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define SPI_IMU_NSS_SET_LOW		 			GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define SPI_IMU_NSS_SET_TOGGLE				GPIOB->ODR ^= GPIO_Pin_0

#define SPI_PRESSURE_NSS_SET_HIGH	 		GPIO_SetBits(GPIOC, GPIO_Pin_15)
#define SPI_PRESSURE_NSS_SET_LOW			GPIO_ResetBits(GPIOC, GPIO_Pin_15)
#define SPI_PRESSURE_NSS_SET_TOGGLE			GPIOC->ODR ^= GPIO_Pin_15

#define SPI_RF_NSS_SET_HIGH	 				GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define SPI_RF_NSS_SET_LOW		 			GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define SPI_RF_NSS_SET_TOGGLE				GPIOB->ODR ^= GPIO_Pin_4

#define SPI_BLDC_NSS_SET_HIGH	 			GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define SPI_BLDC_NSS_SET_LOW		 		GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define SPI_BLDC_NSS_SET_TOGGLE				GPIOC->ODR ^= GPIO_Pin_13

#define SPI_ALL_NSS_SET_HIGH				GPIO_SetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_4); GPIO_SetBits(GPIOC, GPIO_Pin_13|GPIO_Pin_15)

#define SPI_TRANSACTION_BUFF_SIZE			16

typedef enum {
	SPI_SLAVE_SELECT_NONE = 0,
	SPI_SLAVE_SELECT_IMU,
	SPI_SLAVE_SELECT_PRESSURE,
	SPI_SLAVE_SELECT_RF,
	SPI_SLAVE_SELECT_BLDC,
	SPI_SLAVE_SELECT_ENCODER
} SpiSlaveSelect_t;

typedef struct {
	uint8_t *p_tx_buff;
	uint8_t *p_rx_buff;
	SpiSlaveSelect_t slave;
	uint32_t data_length;
	void (*p_cb)(uint8_t *p_rx);
} SpiTransactionRecord_t;

void spi_init(void);

bool spi_get_init_status(void);

void spi_slave_select(SpiSlaveSelect_t slave);

uint32_t spi_get_max_queue_depth(void);

void spi_add_transaction(SpiTransactionRecord_t *p_record);

#endif
