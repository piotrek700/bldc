#ifndef BUZZER_H_
#define BUZZER_H_

#include "platform.h"
#include "buzzer_timer.h"
#include "buzzer_settings.h"

#define BUZZER_GENERATE_ENUM(STR, SOUND) 			BUZZER_SOUND_##STR,
#define BUZZER_GENERATE_DICTIONARY(STR, SOUND) 		{SOUND, sizeof(SOUND)/sizeof(BuzzerSoundStep)},

typedef struct {
	uint16_t generation_time_ms;
	uint16_t frequency_hz;
} BuzzerSoundStep;

typedef enum {
	BUZZER_SOUNDS_LIST(BUZZER_GENERATE_ENUM)
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
