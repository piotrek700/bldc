#include <sdk/buzzer.h>
#include <sdk/buzzer_sounds.h>
#include <sdk/debug.h>
#include <sdk/tick.h>
#include <sdk/rybos.h>
#include "Buzzer/buzzer_timer.h"

static bool init_status = false;

static const BuzzerSoundStep *sound_ptr;
static uint32_t sound_length = 0;
static uint32_t sound_step = 1;

static const BuzzerDictionaryRow sound_dictionary[] = {
		BUZZER_SOUNDS_LIST(BUZZER_GENERATE_DICTIONARY)
};

void buzzer_test(void) {
	if (!DEBUG_TEST_ENABLE) {
		return;
	}
	//TODO Test
}

void buzzer_init(void) {
	buzzer_timer_init();

	buzzer_test();

	init_status = true;
}

bool buzzer_get_init_status(void) {
	return init_status;
}

void buzzer_generate_sound(BuzzerSoundType sound_type) {
	sound_ptr = sound_dictionary[sound_type].sound_ptr;
	sound_length = sound_dictionary[sound_type].sound_length;
	sound_step = 0;

	rybos_task_enable(RYBOS_MARKER_TASK_BUZZER, true);
}

void buzzer_state_machine(void) {
	static uint32_t next_time_step = 0;

	if (sound_step <= sound_length) {
		if (sound_step == 0) {
			next_time_step = tick_get_time_ms() + sound_ptr[sound_step].generation_time_ms;
			buzzer_timer_set_frequency(sound_ptr[sound_step].frequency_hz);
			sound_step++;
		} else {
			if (tick_get_time_ms() >= next_time_step) {
				if (sound_step == sound_length) {
					buzzer_timer_set_frequency(0);
					rybos_task_enable(RYBOS_MARKER_TASK_BUZZER, false);
					return;
				}
				next_time_step += sound_ptr[sound_step].generation_time_ms;
				buzzer_timer_set_frequency(sound_ptr[sound_step].frequency_hz);
				sound_step++;
			}
		}
	}
}

