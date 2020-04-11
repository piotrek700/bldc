#ifndef BUZZER_H_
#define BUZZER_H_

#include "platform.h"

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

typedef struct {
	const BuzzerSoundStep *sound_ptr;
	uint32_t sound_length;
} BuzzerDictionaryRow;

void buzzer_init(void);

void buzzer_test(void);

bool buzzer_get_init_status(void);

void buzzer_generate_sound(BuzzerSoundType sound_type);

void buzzer_state_machine(void);

#endif
