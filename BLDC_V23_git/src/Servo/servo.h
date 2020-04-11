#ifndef SERVO_H_
#define SERVO_H_

#include "platform.h"
#include "../Tick/tick.h"

#define SERVO_CLOCK_PRESCALER	22
#define SERVO_FREQUENCY_HZ		50
#define SERVO_PERIOD			(TICK_CPU_FREQUENCY_HZ/SERVO_FREQUENCY_HZ/SERVO_CLOCK_PRESCALER)
#define SERVO_MAX_ANGLE			80.0f
#define SERVO_MIN_ANGLE			-80.0f

typedef enum {
	SERVO_POSITION_1_LEFT = 0,
	SERVO_POSITION_2_TOP,
	SERVO_POSITION_3_RIGHT,
	SERVO_POSITION_4_BOTTOM
} ServoPosition;

void servo_test(void);

void servo_init(void);

bool servo_get_init_status(void);

void servo_set_position_angle(ServoPosition servo_position, float angle);

float *servo_get_angle(void);

void servo_set_all(float angle);

#endif
