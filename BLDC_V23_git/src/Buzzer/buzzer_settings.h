#ifndef BUZZER_SETTINGS_H_
#define BUZZER_SETTINGS_H_

//List off all available sounds, each enum will star at prefix BUZZER_SOUND_xxx
#define BUZZER_SOUNDS_LIST(SOUND) 					\
	SOUND(START, 		buzzer_sound_start)			\
	SOUND(MARIO, 		buzzer_sound_mario)			\
	SOUND(SINGLE_PEAK, 	buzzer_sound_single_peak)	\
	SOUND(DOUBLE_PEAK, 	buzzer_sound_double_peak)	\
	SOUND(TRIPLE_PEAK, 	buzzer_sound_triple_peak)	\
	SOUND(TURN_OFF, 	buzzer_sound_turn_off)

//Define function which generate a frequency
#define BUZZER_GENERATE_SOUND(FREQ)					buzzer_timer_set_frequency(FREQ)

#endif
