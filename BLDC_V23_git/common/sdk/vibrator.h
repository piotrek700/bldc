#ifndef VIBRATOR_H_
#define VIBRATOR_H_

#include "platform.h"
#include <settings/vibrator_settings.h>

#define VIBRATOR_GENERATE_ENUM(str, sound) 				VIBRATOR_SOUND_##str,
#define VIBRATOR_GENERATE_DICTIONARY(str, sound) 		{sound, sizeof(sound)/sizeof(VIBRATORSoundStep)},

typedef struct {
	uint16_t generation_time_ms;
	bool vibrator_on;
} VibratorSoundStep;

typedef enum {
	VIBRATOR_SOUNDS_LIST(VIBRATOR_GENERATE_ENUM)
} VibratorSoundType;

typedef struct {
	const VibratorSoundStep *sound_ptr;
	uint32_t sound_length;
} VibratorDictionaryRow;

void vibrator_init(void);

void vibrator_test(void);

bool vibrator_get_init_status(void);

void vibrator_generate_sound(VibratorSoundType sound_type);

void vibrator_state_machine(void);

#endif
