#include "rybos.h"
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include "../utils.h"

static volatile RybosTask task_list[RYBOS_NUMBER_OF_TASK];
static volatile TaskLoadStack stack[RYBOS_TASK_MARKER_STACK_SIZE];
static volatile uint32_t exec_time[MARKER_SYSTEM];
static volatile uint32_t exec_cnt[MARKER_SYSTEM + 1];
static volatile uint32_t stack_ptr = 0;
static volatile uint32_t marker_to_id[MARKER_SYSTEM];

void rybos_add_task(uint32_t period, uint32_t priority, uint8_t *description, void (*cb)(void), TaskMarker marker, bool enable) {
	static uint32_t task_cnt = 0;

	if (task_cnt == RYBOS_NUMBER_OF_TASK) {
		debug_error(DEBUG_RYBOS_TASK_LIST_LENGTH_ERROR);
	}

	task_list[task_cnt].period = period;
	task_list[task_cnt].priority = priority;
	task_list[task_cnt].timer = period;	//TODO rand
	task_list[task_cnt].description = description;
	task_list[task_cnt].cb = cb;
	task_list[task_cnt].marker = marker;
	task_list[task_cnt].enable = enable;

	marker_to_id[marker] = task_cnt;

	task_cnt++;
}

void rybos_scheduler_run(void) {
	//TODO iwdg_kick();

	//Find the highest priority task
	static uint32_t active_task_ptr = 0;
	uint32_t last_task = active_task_ptr;
	uint32_t max_priority = RYBOS_LOWEST_PRIORITY;
	uint32_t max_priority_ptr = 0;

	do {
		active_task_ptr++;
		if (active_task_ptr == RYBOS_NUMBER_OF_TASK) {
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

		//TODO Check if cyclic task, Check if refresh required
	} while (last_task != active_task_ptr);

	//Update timer and execute task
	active_task_ptr = max_priority_ptr;
	task_list[active_task_ptr].timer += task_list[active_task_ptr].period;

	rybos_task_start_marker(task_list[active_task_ptr].marker);
	task_list[active_task_ptr].cb();
	rybos_task_stop_marker(task_list[active_task_ptr].marker);

	//Execution counter
	exec_cnt[MARKER_SYSTEM]++;
}

void rybos_task_start_marker(TaskMarker marker) {
	enter_critical();

	//TODO debug, do not remove
	//Check stack overflow
	//if (stack_ptr == RYBOS_TASK_MARKER_STACK_SIZE) {
	//	debug_error_handler(RYBOS_TASK_MARKER_STACK_OVERLOW);
	//}

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

void rybos_task_stop_marker(TaskMarker marker) {
	//Check marker
	enter_critical();

	//asm volatile("" ::: "memory");
	stack_ptr--;

	//TODO debug, do not remove
	//if (stack[stack_ptr].marker != marker) {
	//	debug_error_handler(RYBOS_TASK_MARKER_ERROR);
	//}

	//Get stop time
	uint32_t task_stop_tick = tick_get_clock_tick();

	//Calculate top stack task period
	//Overflow protection
	uint32_t task_execution_time;
	if (task_stop_tick >= stack[stack_ptr].start_cnt) {
		task_execution_time = task_stop_tick - stack[stack_ptr].start_cnt;
	} else {
		task_execution_time = (uint32_t) 0xFFFFFFFF - (stack[stack_ptr].start_cnt - task_stop_tick) + (uint32_t)1;
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

uint32_t rybos_get_task_execution_time(TaskMarker marker) {
	return exec_time[marker];
}

void rybos_clear_task_execution_time(TaskMarker marker) {
	exec_time[marker] = 0;
}

uint32_t rybos_get_task_execution_cnt(TaskMarker marker) {
	return exec_cnt[marker];
}

void rybos_clear_task_execution_cnt(TaskMarker marker) {
	exec_cnt[marker] = 0;
}

void rybos_task_enable(TaskMarker marker, bool enable) {
	enter_critical();

	uint32_t i = marker_to_id[marker];

	task_list[i].enable = enable;
	if (task_list[i].period == 0) {
		task_list[i].timer = 0;
	} else {
		task_list[i].timer = tick_get_time_ms();
	}

	exit_critical();
}

void rybos_task_enable_time(TaskMarker marker, uint32_t timer, bool enable) {
	enter_critical();

	uint32_t i = marker_to_id[marker];

	task_list[i].enable = enable;
	task_list[i].timer = timer;

	exit_critical();
}
