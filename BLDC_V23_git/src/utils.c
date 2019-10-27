#include "utils.h"

static volatile uint32_t critical_cnt = 0;
static volatile uint32_t critical_max = 0;

float inv_sqrtf(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//https://gist.github.com/volkansalma/2972237
float fast_atan2f(float y, float x) {
	if (x == 0.0f) {
		if (y > 0.0f)
			return (float) M_PI_2;
		if (y == 0.0f)
			return 0.0f;
		return -(float) M_PI_2;
	}

	float atan;
	float z = y / x;
	if (fabsf(z) < 1.0f) {
		atan = z / (1.0f + 0.28f * z * z);
		if (x < 0.0f) {
			if (y < 0.0f)
				return atan - (float) M_PI;
			return atan + (float) M_PI;
		}
	} else {
		atan = (float) M_PI_2 - z / (z * z + 0.28f);
		if (y < 0.0f)
			return atan - (float) M_PI;
	}
	return atan;
}

void enter_critical(void) {
	__disable_irq();
	critical_cnt++;	//TODO safe increment

	if (critical_cnt > critical_max) {
		critical_max = critical_cnt;
	}
}

void exit_critical(void) {
	critical_cnt--;	//Todo safe decrement
	if (critical_cnt == 0) {
		__enable_irq();
	}
}

uint32_t critiacl_get_max_queue_depth(void) {
	return critical_max;
}

float fast_log(float val) {
	int32_t * const exp_ptr = (int32_t *) (&val);
	int32_t x = *exp_ptr;
	const int32_t log_2 = ((x >> 23) & 255) - 128;
	x &= ~(255 << 23);
	x += 127 << 23;
	*exp_ptr = x;

	val = ((-1.0f / 3.0f) * val + 2.0f) * val - 2.0f / 3.0f;

	return (val + log_2) * 0.69314718f;
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
