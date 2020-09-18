#ifndef VIBRATOR_SOUNDS_H_
#define VIBRATOR_SOUNDS_H_

#include <sdk/vibrator.h>

static const VibratorSoundStep vibrator_sound_start_3x[] = {
		{ 100, 1 },
		{ 100, 0 },
		{ 100, 1 },
		{ 100, 0 },
		{ 300, 1 }
};

static const VibratorSoundStep vibrator_sound_single[] = {
		{ 100, 1 },
};

#endif
