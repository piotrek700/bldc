#ifndef RYBOS_H_
#define RYBOS_H_

#include <stdint.h>
#include <stdbool.h>
#include "rybos_settings.h"

#define RYBOS_GENERATE_IRQ_ENUM(enum)  				RYBOS_MARKER_IRQ_##enum,
#define RYBOS_GENERATE_TASK_ENUM(enum) 				RYBOS_MARKER_TASK_##enum,

#define RYBOS_GENERATE_IRQ_STRING(str) 				#str,
#define RYBOS_GENERATE_TASK_STRING(str) 			#str,

#define RYBOS_GENERATE_IRQ_UNIQUE_MASK_ENUM(enum)  	RYBOS_IRQ_UNIQUE_MASK_##enum = (1 << RYBOS_MARKER_IRQ_##enum),

#define RYBOS_TASK_IRQ_NAME_MAX_SIZE				16
#define RYBOS_HIGHEST_PRIORITY						0
#define RYBOS_LOWEST_PRIORITY						0xFFFFFFFF
#define RYBOS_TASK_MARKER_STACK_SIZE				8

#define RYBOS_TASK_AND_IRQ_SIZE						(RYBOS_MARKER_TASK_SIZE + RYBOS_MARKER_IRQ_SIZE)

typedef enum {
	//Roll-out to all IRQs
	RYBOS_IRQ_LIST(RYBOS_GENERATE_IRQ_ENUM)
	RYBOS_MARKER_IRQ_SIZE,
	//ENUM decrementation
	RYBOS_NOT_USED = RYBOS_MARKER_IRQ_SIZE - 1,
	//Roll-out to all tasks
	RYBOS_TASK_LIST(RYBOS_GENERATE_TASK_ENUM)
	RYBOS_MARKER_TASK_SYSTEM,
	RYBOS_MARKER_TASK_SIZE = RYBOS_MARKER_TASK_SYSTEM - RYBOS_NOT_USED,
} RybosIrqTaskMarker;

//Generate IRQ and task names
static const uint8_t RYBOS_IRQ_TASK_NAMES[][RYBOS_TASK_IRQ_NAME_MAX_SIZE] = {
		//Roll-out to all IRQs
		RYBOS_IRQ_LIST(RYBOS_GENERATE_IRQ_STRING)
		//Roll-out to all tasks
		RYBOS_TASK_LIST(RYBOS_GENERATE_TASK_STRING)
		//System task
		"SYSTEM"
};

//Generate IRQ unique masks
typedef enum {
	//Roll-out to all IRQs
	RYBOS_IRQ_LIST(RYBOS_GENERATE_IRQ_UNIQUE_MASK_ENUM)
	}RybosIrqUniqueMask;

typedef struct {
	uint32_t period;
	uint32_t priority;
	uint32_t timer;
	void (*cb)(void);
	bool enable;
} RybosTask;

typedef struct {
	uint32_t start_cnt;
	uint32_t preemption_time;
	RybosIrqTaskMarker marker;
} RybosLoadStack;

void rybos_add_task(uint32_t period, uint32_t priority, void (*cb)(void), RybosIrqTaskMarker marker, bool enable);

void rybos_scheduler_run(void);

void rybos_task_start_marker(RybosIrqTaskMarker marker);

void rybos_task_stop_marker(RybosIrqTaskMarker marker);

uint32_t rybos_get_task_execution_time(RybosIrqTaskMarker marker);

void rybos_clear_task_execution_time(RybosIrqTaskMarker marker);

uint32_t rybos_get_task_execution_cnt(RybosIrqTaskMarker marker);

void rybos_clear_task_execution_cnt(RybosIrqTaskMarker marker);

uint32_t rybos_get_irq_execution_mask(void);

void rybos_clear_irq_execution_mask(void);

void rybos_task_enable(RybosIrqTaskMarker marker, bool enable);

void rybos_task_enable_time(RybosIrqTaskMarker marker, uint32_t timer, bool enable);

#endif
