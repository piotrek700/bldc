#include "spi.h"
#include <sdk/debug.h>
#include <sdk/rybos.h>
#include <sdk/cyclic_ptr.h>
#include <sdk/atomic.h>

static bool init_status = false;
static volatile SpiTransactionRecord *previous_transaction = 0;
static volatile SpiTransactionRecord *active_transaction = 0;

CYCLIC_BUFFER_PTR_DEF(spi_cyclic, false, SPI_TRANSACTION_BUFF_SIZE);

static void spi_gpio_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	//SCK	PA5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	//MOSI  PA7
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
	//MISO	PA6
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//MISO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//IMU NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//RF NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//BLDC NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//PRESSURE NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void spi_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(DMA1_Channel2_IRQn, 5);
}

static void spi_spi1_init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_I2S_DeInit(SPI1);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	SPI_Cmd(SPI1, ENABLE);
}

static void spi_dma_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel2);
	DMA_DeInit(DMA1_Channel3);

	//RX
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;
	DMA_InitStructure.DMA_BufferSize = (uint16_t) 0;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

	//TX
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI1->DR;
	DMA_InitStructure.DMA_BufferSize = (uint16_t) 0;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
}

void spi_init(void) {
	spi_gpio_init();
	spi_dma_init();
	spi_nvic_init();
	spi_spi1_init();

	spi_slave_select(SPI_SLAVE_SELECT_NONE);

	spi_test();

	init_status = true;
}

void spi_slave_select(SpiSlaveSelect slave) {
	//Press, CPOL H, 2Edge
	//IMU,   CPOL H, 2Edge
	//RF,    CPOL L, 1Edge
	//BLDC,  CPOL L, 2Edge

	//Disable all CS
	SPI_ALL_NSS_SET_HIGH;

	switch (slave) {
	case SPI_SLAVE_SELECT_NONE:
		break;

	case SPI_SLAVE_SELECT_IMU:
		//SPI Disable
		SPI1->CR1 &= (uint16_t) (~SPI_CR1_SPE);
		SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
		//SPI Enable
		SPI1->CR1 |= SPI_CR1_SPE;

		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

		SPI_IMU_NSS_SET_LOW;

		break;

	case SPI_SLAVE_SELECT_PRESSURE:
		//SPI Disable
		SPI1->CR1 &= (uint16_t) (~SPI_CR1_SPE);
		SPI1->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
		//SPI Enable
		SPI1->CR1 |= SPI_CR1_SPE;

		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

		SPI_PRESSURE_NSS_SET_LOW;
		break;

	case SPI_SLAVE_SELECT_RF:
		//SPI Disable
		SPI1->CR1 &= (uint16_t) (~SPI_CR1_SPE);
		SPI1->CR1 &= ~(SPI_CPOL_High | SPI_CPHA_2Edge);
		//SPI Enable
		SPI1->CR1 |= SPI_CR1_SPE;

		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

		SPI_RF_NSS_SET_LOW;
		break;

	case SPI_SLAVE_SELECT_BLDC:
		//SPI Disable
		SPI1->CR1 &= (uint16_t) (~SPI_CR1_SPE);
		SPI1->CR1 &= ~SPI_CPOL_High;
		SPI1->CR1 |= SPI_CPHA_2Edge;
		//SPI Enable
		SPI1->CR1 |= SPI_CR1_SPE;

		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

		SPI_BLDC_NSS_SET_LOW;
		break;

	case SPI_SLAVE_SELECT_ENCODER:
		//SPI Disable
		SPI1->CR1 &= (uint16_t) (~SPI_CR1_SPE);
		SPI1->CR1 &= ~(SPI_CPOL_High);
		SPI1->CR1 |= SPI_CPHA_2Edge;
		//SPI Enable
		SPI1->CR1 |= SPI_CR1_SPE;

		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");

		SPI_RF_NSS_SET_LOW;
		break;

	default:
		debug_error(SLAVE_SELECTED_NOT_SUPPORTED);
		break;
	}
}

bool spi_get_init_status(void) {
	return init_status;
}

void spi_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

static void spi_dma_start_next_transation(void) {
	//DMA clear flag
	DMA1->IFCR = DMA1_FLAG_GL2 | DMA1_FLAG_GL3;

	//DMA disable
	DMA1_Channel2->CCR &= (uint16_t) (~DMA_CCR_EN);
	DMA1_Channel3->CCR &= (uint16_t) (~DMA_CCR_EN);

	//SPI DMA disable
	SPI1->CR2 &= (uint16_t) (~(SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx));

	//DMA1_Channel2->CNDTR = active_transaction->data_length;
	//DMA1_Channel2->CMAR = (uint32_t) active_transaction->rx_buff;

	DMA1_Channel3->CNDTR = active_transaction->data_length;
	DMA1_Channel3->CMAR = (uint32_t) active_transaction->tx_buff;

	//Handle null rx buffer
	if (active_transaction->rx_buff == 0) {
		static uint8_t null[1];

		//Redirect received data to null
		DMA1_Channel2->CNDTR = active_transaction->data_length;
		DMA1_Channel2->CMAR = (uint32_t) null;

		//memory increment off
		DMA1_Channel2->CCR &= ~ DMA_CCR_MINC;
	} else {
		DMA1_Channel2->CNDTR = active_transaction->data_length;
		DMA1_Channel2->CMAR = (uint32_t) active_transaction->rx_buff;

		//memory increment on
		DMA1_Channel2->CCR |= DMA_CCR_MINC;
	}

	//Slave select
	spi_slave_select(active_transaction->slave);

	//SPI DMA enable
	SPI1->CR2 |= SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

	//DMA enable
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

uint32_t spi_get_max_queue_depth(void) {
	return cyclic_ptr_get_max_elements((CyclicPtrBuffer *) &spi_cyclic);
}

void spi_add_transaction(SpiTransactionRecord *record) {
	enter_critical();

	cyclic_ptr_add((CyclicPtrBuffer *) &spi_cyclic, record);

	//Check if DMA disabled
	if (!(DMA1_Channel3->CCR & DMA_CCR_EN)) {
		cyclic_ptr_get((CyclicPtrBuffer *) &spi_cyclic, (type_buff *) &active_transaction);
		spi_dma_start_next_transation();
	}
	exit_critical();
}

//SPI RX
void DMA1_Channel2_IRQHandler(void) {
	//TODO speedup this sequence by replacing all DMA by direct register access
	rybos_task_start_marker(RYBOS_MARKER_IRQ_SPI_DMA);

	if (DMA_GetITStatus(DMA1_IT_TC2) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC2);

		spi_slave_select(SPI_SLAVE_SELECT_NONE);

		if (spi_cyclic.elements == 0) {
			//Disable DMA
			DMA_Cmd(DMA1_Channel2, DISABLE);
			DMA_Cmd(DMA1_Channel3, DISABLE);

			if (active_transaction->cb != 0) {
				active_transaction->cb(active_transaction->rx_buff);
			}
		} else {
			previous_transaction = active_transaction;
			cyclic_ptr_get((CyclicPtrBuffer *) &spi_cyclic, (type_buff *) &active_transaction);

			//Execute cb
			if (previous_transaction == active_transaction) {
				if (previous_transaction->cb != 0) {
					previous_transaction->cb(previous_transaction->rx_buff);
				}

				spi_dma_start_next_transation();
			} else {

				spi_dma_start_next_transation();
				if (previous_transaction->cb != 0) {
					previous_transaction->cb(previous_transaction->rx_buff);
				}
			}
		}
	}
	rybos_task_stop_marker(RYBOS_MARKER_IRQ_SPI_DMA);
}
