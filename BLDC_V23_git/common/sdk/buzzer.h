#ifndef BUZZER_H_
#define BUZZER_H_

#include "platform.h"
#include <settings/buzzer_settings.h>

#define BUZZER_GENERATE_ENUM(str, sound) 			BUZZER_SOUND_##str,
#define BUZZER_GENERATE_DICTIONARY(str, sound) 		{sound, sizeof(sound)/sizeof(BuzzerSoundStep_t)},

typedef struct {
	uint16_t generation_time_ms;
	uint16_t frequency_hz;
} BuzzerSoundStep_t;

typedef enum {
	BUZZER_SOUNDS_LIST(BUZZER_GENERATE_ENUM)
} BuzzerSoundType_t;

typedef struct {
	const BuzzerSoundStep_t *sound_ptr;
	uint32_t sound_length;
} BuzzerDictionaryRow_t;

void buzzer_init(void);

void buzzer_test(void);

bool buzzer_get_init_status(void);

void buzzer_generate_sound(BuzzerSoundType_t sound_type);

void buzzer_state_machine(void);

#endif
