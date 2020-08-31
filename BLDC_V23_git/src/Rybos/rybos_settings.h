#ifndef RYBOS_SETTINGS_H_
#define RYBOS_SETTINGS_H_

//IRQ list definition
#define RYBOS_IRQ_LIST(irq) 	\
		irq(TICK)				\
		irq(SPI_DMA)			\
		irq(DRV_FAULT)			\
		irq(ADC_BLDC)			\
		irq(UART_DMA)			\
		irq(SI4468_NIRQ)		\
		irq(SI4468_CTS)

//Task list definition
#define RYBOS_TASK_LIST(task) 	\
		task(LED)				\
		task(SLEEP)				\
		task(LOAD_MONITOR)		\
		task(BUZZER)			\
		task(PRESSURE_READ)		\
		task(IMU_READ)			\
		task(BLDC_STATUS)		\
		task(FRAME_DECODER)		\
		task(RF)				\
		task(PARAM_UPDATE)		\
		task(LOGGER)			\
		task(RF_TIMEOUT)

#endif
