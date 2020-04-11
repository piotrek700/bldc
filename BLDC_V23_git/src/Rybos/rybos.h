#ifndef RYBOS_H_
#define RYBOS_H_

#include <stdint.h>
#include <stdbool.h>

#define RYBOS_NUMBER_OF_TASK			12
#define RYBOS_HIGHEST_PRIORITY			0
#define RYBOS_LOWEST_PRIORITY			0xFFFFFFFF
#define RYBOS_TASK_MARKER_STACK_SIZE	20
#define RYBOS_TASK_NAME_MAX_SIZE		32

typedef enum {
	MARKER_IRQ_TICK = 0,			//0
	MARKER_TASK_LED,				//1
	MARKER_TASK_SLEEP,				//2
	MARKER_TASK_LOAD_MONITOR,		//3
	MARKER_TASK_BUZZER,				//4
	MARKER_TASK_PRESSURE_READ,		//5
	MARKER_IRQ_SPI_DMA,				//6
	MARKER_TASK_IMU_READ,			//7
	MARKER_IRQ_DRV_FAULT,			//8
	MARKER_TASK_BLDC_STATUS,		//9
	MARKER_IRQ_ADC_NTC,				//10
	MARKER_IRQ_ADC_BLDC,			//11
	MARKER_TASK_FRAME_DECODER,		//12
	MARKER_TASK_RF,					//13
	MARKER_TASK_PARAM_UPDATE, 		//14
	MARKER_TASK_LOGGER,				//15
	MARKER_TASK_RF_TIMEOUT,			//16
	MARKER_IRQ_UART_DMA,			//17
	MARKER_IRQ_SI4468_NIRQ,			//18
	MARKER_IRQ_SI4468_CTS,			//19
	MARKER_SYSTEM					//20 RYBOS
} TaskMarker;

static const uint8_t TASK_NAMES[][RYBOS_TASK_NAME_MAX_SIZE] ={		//TODO divide into separate array
	"IRQ_TICK",						//0
	"TASK_LED",						//1
	"TASK_SLEEP",					//2
	"TASK_LOAD_MONITOR",			//3
	"TASK_BUZZER",					//4
	"TASK_PRESSURE_READ",			//5
	"IRQ_SPI_DMA",					//6
	"TASK_IMU_READ",				//7
	"IRQ_DRV_FAULT",				//8
	"TASK_BLDC_STATUS",				//9
	"IRQ_ADC_NTC",					//10
	"IRQ_ADC_BLDC",					//11
	"TASK_FRAME_DECODER",			//12
	"TASK_RF",						//13
	"TASK_PARAM_UPDATE", 			//14
	"TASK_LOGGER",					//15
	"TASK_RF_TIMEOUT",				//16
	"IRQ_UART_DMA",					//17
	"IRQ_SI4468_NIRQ",				//18
	"IRQ_SI4468_CTS",				//19
	"TASK_SYSTEM"					//20 RYBOS
};

typedef struct {
	uint32_t period;
	uint32_t priority;
	uint32_t timer;
	uint8_t *description;
	void (*cb)(void);
	TaskMarker marker;
	bool enable;
} RybosTask;

typedef struct {
	uint32_t start_cnt;
	uint32_t preemption_time;
	TaskMarker marker;
} TaskLoadStack;

void rybos_add_task(uint32_t period, uint32_t priority, uint8_t *description, void (*cb)(void), TaskMarker marker, bool enable);

void rybos_scheduler_run(void);

void rybos_task_start_marker(TaskMarker marker);

void rybos_task_stop_marker(TaskMarker marker);

uint32_t rybos_get_task_execution_time(TaskMarker marker);

void rybos_clear_task_execution_time(TaskMarker marker);

uint32_t rybos_get_task_execution_cnt(TaskMarker marker);

void rybos_clear_task_execution_cnt(TaskMarker marker);

void rybos_task_enable(TaskMarker marker, bool enable);

void rybos_task_enable_time(TaskMarker marker, uint32_t timer, bool enable);

#endif
