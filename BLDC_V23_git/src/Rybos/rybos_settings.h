#ifndef RYBOS_SETTINGS_H_
#define RYBOS_SETTINGS_H_

//IRQ list definition
#define RYBOS_IRQ_LIST(IRQ) 	\
		IRQ(TICK)				\
		IRQ(SPI_DMA)			\
		IRQ(DRV_FAULT)			\
		IRQ(ADC_NTC)			\
		IRQ(ADC_BLDC)			\
		IRQ(UART_DMA)			\
		IRQ(SI4468_NIRQ)		\
		IRQ(SI4468_CTS)			\

//Task list definition
#define RYBOS_TASK_LIST(TASK) 	\
		TASK(LED)				\
		TASK(SLEEP)				\
		TASK(LOAD_MONITOR)		\
		TASK(BUZZER)			\
		TASK(PRESSURE_READ)		\
		TASK(IMU_READ)			\
		TASK(BLDC_STATUS)		\
		TASK(FRAME_DECODER)		\
		TASK(RF)				\
		TASK(PARAM_UPDATE)		\
		TASK(LOGGER)			\
		TASK(RF_TIMEOUT)		\

#endif
