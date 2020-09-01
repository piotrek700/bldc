#include "atomic.h"
#include "../utils.h"

CCMRAM_VARIABLE static volatile uint32_t critical_cnt = 0;
CCMRAM_VARIABLE static volatile uint32_t critical_max = 0;

CCMRAM_FUCNTION void enter_critical(void) {
	//Mask priority, 4 is a ADC priority must be shifted by 4 x<<4
	//__set_BASEPRI(4<<4);

	__disable_irq();
	critical_cnt++;

	if (critical_cnt > critical_max) {
		critical_max = critical_cnt;
	}
}

CCMRAM_FUCNTION void exit_critical(void) {
	critical_cnt--;
	if (critical_cnt == 0) {
		__enable_irq();

		//Mask priority
		//__set_BASEPRI(0);
	}
}

uint32_t critiacl_get_max_queue_depth(void) {
	return critical_max;
}

void safe_increment(uint32_t *addr) {
	uint32_t new_value;
	do {
		new_value = __LDREXW(addr) + 1;
	} while (__STREXW(new_value, addr));
}

void safe_decrement(uint32_t *addr) {
	uint32_t new_value;
	do {
		new_value = __LDREXW(addr) - 1;
	} while (__STREXW(new_value, addr));
}
