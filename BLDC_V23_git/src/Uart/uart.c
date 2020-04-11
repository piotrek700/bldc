#include "uart.h"
#include "../Debug/debug.h"
#include "../Cyclic/cyclic.h"
#include "../Frame/frame.h"
#include "../Rybos/rybos.h"
#include "utils.h"

static bool init_status = false;

static uint8_t uart_rx_dma_register[UART_DMA_RX_BUFFER_LENGTH];

CYCLIC_BUFFER_DEF(uart_cyclic, false, UART_FRAME_QUEUE_SIZE, sizeof(UartFrame));

static volatile uint32_t tx_bytes_cnt = 0;
static volatile uint32_t rx_bytes_cnt = 0;
static volatile uint32_t rx_error_frames_cnt = 0;
static volatile uint32_t recived_frame_cnt = 0;
static volatile uint32_t transmitted_frame_cnt = 0;

void uart_reset_statistics(void) {
	tx_bytes_cnt = 0;
	rx_bytes_cnt = 0;
	rx_error_frames_cnt = 0;
	recived_frame_cnt = 0;
	transmitted_frame_cnt = 0;
}

uint32_t uart_get_transmitted_bytes(void) {
	return tx_bytes_cnt;
}

uint32_t uart_get_received_bytes(void) {
	return rx_bytes_cnt;
}

uint32_t uart_get_received_error_frames(void) {
	return rx_error_frames_cnt;
}

uint32_t uart_get_received_frames(void) {
	return recived_frame_cnt;
}

void uart_increment_received_error_frame_cnt(void) {
	rx_error_frames_cnt++;
}

void uart_increment_reveived_frame_cnt(void) {
	recived_frame_cnt++;
}

uint32_t uart_get_transmitted_frames(void) {
	return transmitted_frame_cnt;
}

static void uart_gpio_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

	//RX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void uart_nvic_init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(DMA1_Channel7_IRQn, 8);
}

static void uart_uart2_init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_DeInit(USART2);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 3000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_OverSampling8Cmd(USART2, ENABLE);
	USART_Init(USART2, &USART_InitStructure);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	//USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

	USART_Cmd(USART2, ENABLE);
}

static void uart_dma_init(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel6);
	DMA_DeInit(DMA1_Channel7);

	//RX
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->RDR;
	DMA_InitStructure.DMA_BufferSize = (uint16_t) UART_DMA_RX_BUFFER_LENGTH;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) uart_rx_dma_register;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel6, ENABLE);

	//TX
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART2->TDR;
	DMA_InitStructure.DMA_BufferSize = (uint16_t) 0;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA1_Channel7, ENABLE);
}

void uart_init(void) {
	uart_gpio_init();
	uart_dma_init();
	uart_nvic_init();
	uart_uart2_init();
	uart_test();

	init_status = true;
}

void uart_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

bool uart_get_init_status(void) {
	return init_status;
}

bool uart_get_byte_dma(uint8_t *data) {
	static uint32_t dma_read_ptr = 0;

	if (UART_DMA_RX_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA1_Channel6) != dma_read_ptr) {
		*data = uart_rx_dma_register[dma_read_ptr];
		dma_read_ptr++;
		if (dma_read_ptr == UART_DMA_RX_BUFFER_LENGTH) {
			dma_read_ptr = 0;
		}
		rx_bytes_cnt++;
		return true;
	}
	return false;
}

uint32_t uart_get_max_queue_depth(void) {
	return cyclic_get_max_elements((CyclicBuffer *) &uart_cyclic);
}

static void uart_dma_start_next_transation(void) {
	UartFrame *active_transaction = 0;
	cyclic_get((CyclicBuffer *) &uart_cyclic, (uint8_t **) &active_transaction);

	//DMA clear flag
	DMA1->IFCR = DMA1_FLAG_GL7;

	//DMA disable
	DMA1_Channel7->CCR &= (uint16_t) (~DMA_CCR_EN);

	//UART DMA disable
	USART2->CR3 &= (uint16_t) (~USART_DMAReq_Tx);

	DMA1_Channel7->CNDTR = active_transaction->length;
	DMA1_Channel7->CMAR = (uint32_t) active_transaction->tx_buff;

	//TODO remove
	//if(active_transaction->tx_buff < XXX || active_transaction->tx_buff > XXX){
	//	while(1);
	//}

	tx_bytes_cnt += active_transaction->length;
	transmitted_frame_cnt++;

	//UART DMA enable
	USART2->CR3 |= USART_DMAReq_Tx;

	//DMA enable
	DMA1_Channel7->CCR |= DMA_CCR_EN;
}

void uart_send(uint8_t frame_type, uint8_t *frame, uint32_t frame_len) {
	UartFrame *frame_buff;
	uint32_t len = 0;
	uint8_t crc_tmp;

	enter_critical();

	frame_buff = (UartFrame *) cyclic_get_to_add((CyclicBuffer *) &uart_cyclic);

	//Start symbol
	frame_buff->tx_buff[len] = FRAME_START_SYMBOL;
	len++;

	//Type
	if (frame_type == FRAME_START_SYMBOL) {
		frame_buff->tx_buff[len] = frame_type;
		len++;
	}
	frame_buff->tx_buff[len] = frame_type;
	len++;
	crc_tmp = (uint8_t) frame_type;

	//Payload
	uint32_t i;
	for (i = 0; i < frame_len; i++) {
		if (frame[i] == FRAME_START_SYMBOL) {
			frame_buff->tx_buff[len] = frame[i];
			len++;
		}
		frame_buff->tx_buff[len] = frame[i];
		len++;
		crc_tmp += (uint8_t) frame[i];
	}

	//CRC
	if (crc_tmp == FRAME_START_SYMBOL) {
		frame_buff->tx_buff[len] = crc_tmp;
		len++;
	}
	frame_buff->tx_buff[len] = crc_tmp;
	len++;

	//Set length
	frame_buff->length = len;

	if (len >= UART_FRAME_TX_BUFF_SIZE) {
		debug_error(UART_FRAME_MESSAGE_OVERLENGTH);
		return;
	}

	cyclic_move((CyclicBuffer *) &uart_cyclic);

	//Check if DMA disabled
	if (!(DMA1_Channel7->CCR & DMA_CCR_EN)) {
		uart_dma_start_next_transation();
	}

	exit_critical();
}

//UART TX
void DMA1_Channel7_IRQHandler(void) {
	rybos_task_start_marker(MARKER_IRQ_UART_DMA);

	if (DMA_GetITStatus(DMA1_IT_TC7) != RESET) {
		DMA_ClearITPendingBit(DMA1_IT_TC7);

		if (uart_cyclic.elements == 0) {
			//Disable DMA
			DMA_Cmd(DMA1_Channel7, DISABLE);
		} else {
			uart_dma_start_next_transation();
		}
	}
	rybos_task_stop_marker(MARKER_IRQ_UART_DMA);
}
