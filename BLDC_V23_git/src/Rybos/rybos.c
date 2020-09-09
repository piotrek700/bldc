#include "rybos.h"
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../utils.h"
#include "../Atomic/atomic.h"

CCMRAM_VARIABLE static volatile RybosTask task_list[RYBOS_MARKER_TASK_SIZE - 1];
CCMRAM_VARIABLE static volatile RybosLoadStack stack[RYBOS_TASK_MARKER_STACK_SIZE];
CCMRAM_VARIABLE static volatile uint32_t exec_time[RYBOS_TASK_AND_IRQ_SIZE - 1];
//RYBOS_TASK_AND_IRQ_SIZE + System task
CCMRAM_VARIABLE static volatile uint32_t exec_cnt[RYBOS_TASK_AND_IRQ_SIZE];
CCMRAM_VARIABLE static volatile uint32_t stack_ptr = 0;
CCMRAM_VARIABLE static volatile uint32_t iqr_execution_mask = 0;

void rybos_add_task(uint32_t period, uint32_t priority, void (*cb)(void), RybosIrqTaskMarker marker, bool enable) {
	//Cancel enum IRQ offset
	uint32_t i = marker - RYBOS_MARKER_IRQ_SIZE;

	if (i == (RYBOS_MARKER_TASK_SIZE - 1)) {
		debug_error(RYBOS_TASK_LIST_OUT_OF_RANGE);
	}

	task_list[i].period = period;
	task_list[i].priority = priority;
	//TODO random start time generation
	task_list[i].timer = period;
	task_list[i].cb = cb;
	task_list[i].enable = enable;
}

CCMRAM_FUCNTION void rybos_scheduler_run(void) {
	//TODO Watchdog enable
	//iwdg_kick();

	//Find the highest priority task
	static uint32_t active_task_ptr = 0;
	uint32_t last_task = active_task_ptr;
	uint32_t max_priority = RYBOS_LOWEST_PRIORITY;
	uint32_t max_priority_ptr = 0;

	do {
		active_task_ptr++;
		if (active_task_ptr == (RYBOS_MARKER_TASK_SIZE - 1)) {
			active_task_ptr = 0;
		}

		//Check if period finished
		if (task_list[active_task_ptr].enable) {
			if (tick_get_time_ms() > task_list[active_task_ptr].timer) {
				if (task_list[active_task_ptr].priority < max_priority) {
					max_priority = task_list[active_task_ptr].priority;
					max_priority_ptr = active_task_ptr;
				}
			}
		}
	} while (last_task != active_task_ptr);

	//Update timer and execute task
	active_task_ptr = max_priority_ptr;
	task_list[active_task_ptr].timer += task_list[active_task_ptr].period;

	rybos_task_start_marker(active_task_ptr + RYBOS_MARKER_IRQ_SIZE);
	task_list[active_task_ptr].cb();
	rybos_task_stop_marker(active_task_ptr + RYBOS_MARKER_IRQ_SIZE);

	//Execution counter
	exec_cnt[RYBOS_TASK_AND_IRQ_SIZE - 1]++;
}

CCMRAM_FUCNTION void rybos_task_start_marker(RybosIrqTaskMarker marker) {
	enter_critical();

	//Check stack overflow
	if (stack_ptr == RYBOS_TASK_MARKER_STACK_SIZE) {
		debug_error(RYBOS_TASK_MARKER_STACK_OVERLOW);
	}

	//Get start tick time
	stack[stack_ptr].start_cnt = tick_get_clock_tick();
	//Save marker ID
	stack[stack_ptr].marker = marker;
	//Clear preemption time
	stack[stack_ptr].preemption_time = 0;
	//Move stack ptr
	stack_ptr++;

	exit_critical();
}

CCMRAM_FUCNTION void rybos_task_stop_marker(RybosIrqTaskMarker marker) {
	//Check marker
	enter_critical();

	stack_ptr--;

	//IRQ section
	if (marker < RYBOS_MARKER_IRQ_SIZE) {
		//Mark IRQ execution flag
		iqr_execution_mask |= (1 << marker);

		//Lazy stacking detection
		//if ((FPU->FPCCR & FPU_FPCCR_LSPACT_Msk) == 0) {
		//	lazy_stacking_cnt[marker]++;
		//}
	}

	//Get stop time
	uint32_t task_stop_tick = tick_get_clock_tick();

	//Calculate top stack task period
	//Overflow protection
	uint32_t task_execution_time;
	if (task_stop_tick >= stack[stack_ptr].start_cnt) {
		task_execution_time = task_stop_tick - stack[stack_ptr].start_cnt;
	} else {
		task_execution_time = (uint32_t) 0xFFFFFFFF - (stack[stack_ptr].start_cnt - task_stop_tick) + (uint32_t) 1;
	}

	if (stack_ptr > 0) {
		stack[stack_ptr - 1].preemption_time += task_execution_time;
	}

	//Cancel preemption time
	task_execution_time -= stack[stack_ptr].preemption_time;

	//Execution counter
	exec_cnt[marker]++;

	//Accumulate execution time
	exec_time[marker] += task_execution_time;

	exit_critical();
}

CCMRAM_FUCNTION uint32_t rybos_get_task_execution_time(RybosIrqTaskMarker marker) {
	return exec_time[marker];
}

CCMRAM_FUCNTION void rybos_clear_task_execution_time(RybosIrqTaskMarker marker) {
	exec_time[marker] = 0;
}

CCMRAM_FUCNTION uint32_t rybos_get_task_execution_cnt(RybosIrqTaskMarker marker) {
	return exec_cnt[marker];
}

CCMRAM_FUCNTION void rybos_clear_task_execution_cnt(RybosIrqTaskMarker marker) {
	exec_cnt[marker] = 0;
}

CCMRAM_FUCNTION uint32_t rybos_get_irq_execution_mask(void) {
	return iqr_execution_mask;
}

CCMRAM_FUCNTION void rybos_clear_irq_execution_mask(void) {
	iqr_execution_mask = 0;
}

CCMRAM_FUCNTION void rybos_task_enable(RybosIrqTaskMarker marker, bool enable) {
	//Cancel enum IRQ offset
	marker -= RYBOS_MARKER_IRQ_SIZE;

	enter_critical();

	task_list[marker].enable = enable;
	if (task_list[marker].period == 0) {
		task_list[marker].timer = 0;
	} else {
		task_list[marker].timer = tick_get_time_ms();
	}

	exit_critical();
}

CCMRAM_FUCNTION void rybos_task_enable_time(RybosIrqTaskMarker marker, uint32_t timer, bool enable) {
	//Cancel enum IRQ offset
	marker -= RYBOS_MARKER_IRQ_SIZE;

	enter_critical();

	task_list[marker].enable = enable;
	task_list[marker].timer = timer;

	exit_critical();
}
