#ifndef VIBRATOR_H_
#define VIBRATOR_H_

#include "platform.h"
#include <settings/vibrator_settings.h>

#define VIBRATOR_GENERATE_ENUM(str, sound) 				VIBRATOR_SOUND_##str,
#define VIBRATOR_GENERATE_DICTIONARY(str, sound) 		{sound, sizeof(sound)/sizeof(VibratorSoundStep_t)},

typedef struct {
	uint16_t generation_time_ms;
	bool vibrator_on;
} VibratorSoundStep_t;

typedef enum {
	VIBRATOR_SOUNDS_LIST(VIBRATOR_GENERATE_ENUM)
} VibratorSoundType_t;

typedef struct {
	const VibratorSoundStep_t *p_sound;
	uint32_t sound_length;
} VibratorDictionaryRow_t;

void vibrator_init(void);

bool vibrator_get_init_status(void);

void vibrator_generate_sound(VibratorSoundType_t sound_type);

void vibrator_state_machine(void);

#endif
