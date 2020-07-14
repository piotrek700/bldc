#include "utils.h"

CCMRAM_VARIABLE static volatile uint32_t critical_cnt = 0;
CCMRAM_VARIABLE static volatile uint32_t critical_max = 0;

float inv_sqrtf(float x) {
	//TODO replace by sqrt from arm
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
CCMRAM_FUCNTION float fast_atan2f(float y, float x) {
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

float fast_atan2f_sec(float y, float x) {
	float abs_y = fabsf(y) + 1e-20f; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + ((float) M_PI / 4.0f);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * (float) M_PI / 4.0f);
	}

	if (y < 0) {
		return (-angle);
	} else {
		return (angle);
	}
}


CCMRAM_FUCNTION void enter_critical(void) {
	//__disable_irq();
	__set_BASEPRI(4<<4); //4 is a ADC pririty must be shifted by 4 x<<4
	critical_cnt++;	//TODO safe increment

	if (critical_cnt > critical_max) {
		critical_max = critical_cnt;
	}
}

CCMRAM_FUCNTION void exit_critical(void) {
	critical_cnt--;	//Todo safe decrement
	if (critical_cnt == 0) {
		//__enable_irq();
		__set_BASEPRI(0);
	}
}

uint32_t critiacl_get_max_queue_depth(void) {
	return critical_max;
}

CCMRAM_FUCNTION float fast_log(float val) {
	int32_t * const exp_ptr = (int32_t *) (&val);
	int32_t x = *exp_ptr;
	const int32_t log_2 = ((x >> 23) & 255) - 128;
	x &= ~(255 << 23);
	x += 127 << 23;
	*exp_ptr = x;

	val = ((-1.0f / 3.0f) * val + 2.0f) * val - (2.0f / 3.0f);

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
