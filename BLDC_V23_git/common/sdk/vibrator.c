#include "Vibrator/vibrator_drv.h"
#include <sdk/vibrator.h>
#include <sdk/vibrator_sounds.h>
#include <sdk/debug.h>
#include <sdk/tick.h>
#include <sdk/rybos.h>

static bool init_status = false;

static const VibratorSoundStep_t *sound_ptr;
static uint32_t sound_length = 0;
static uint32_t sound_step = 1;

static const VibratorDictionaryRow_t sound_dictionary[] = {
		VIBRATOR_SOUNDS_LIST(VIBRATOR_GENERATE_DICTIONARY)
};

void vibrator_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

void vibrator_init(void) {
	vibrator_drv_init();

	vibrator_test();

	init_status = true;
}

bool vibrator_get_init_status(void) {
	return init_status;
}

void vibrator_generate_sound(VibratorSoundType_t sound_type) {
	sound_ptr = sound_dictionary[sound_type].sound_ptr;
	sound_length = sound_dictionary[sound_type].sound_length;
	sound_step = 0;

	rybos_task_enable(RYBOS_MARKER_TASK_VIBRATOR, true);
}

static void vibrator_set_on_off(bool on) {
	if (on) {
		VIBRATOR_DRV_ON;
	} else {
		VIBRATOR_DRV_OFF;
	}
}

void vibrator_state_machine(void) {
	static uint32_t next_time_step = 0;

	if (sound_step <= sound_length) {
		if (sound_step == 0) {
			next_time_step = tick_get_time_ms() + sound_ptr[sound_step].generation_time_ms;
			vibrator_set_on_off(sound_ptr[sound_step].vibrator_on);
			sound_step++;
		} else {
			if (tick_get_time_ms() >= next_time_step) {
				if (sound_step == sound_length) {
					vibrator_set_on_off(false);
					rybos_task_enable(RYBOS_MARKER_TASK_VIBRATOR, false);
					return;
				}
				next_time_step += sound_ptr[sound_step].generation_time_ms;
				vibrator_set_on_off(sound_ptr[sound_step].vibrator_on);
				sound_step++;
			}
		}
	}
}

