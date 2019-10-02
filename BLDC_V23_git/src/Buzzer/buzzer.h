#ifndef BUZZER_H_
#define BUZZER_H_

#include "stm32f30x.h"
#include "../Debug/debug.h"
#include "../Tick/tick.h"
#include <stdbool.h>
#include "../Rybos/rybos.h"

typedef struct {
	uint32_t generation_time_ms;
	uint32_t frequency_hz;
} BuzzerSoundStep;

typedef enum {
	BUZZER_SOUND_START = 0,
	BUZZER_SOUND_MARIO,
	BUZZER_SOUND_SINGLE_PEAK,
	BUZZER_SOUND_DOUBLE_PEAK,
	BUZZER_SOUND_TRIPLE_PEAK,
	BUZZER_SOUND_TURN_OFF
} BuzzerSoundType;

void buzzer_init(void);

void buzzer_test(void);

bool buzzer_get_init_status(void);

void buzzer_generate_sound(BuzzerSoundType sound_type);

void buzzer_state_machine(void);

#endif
