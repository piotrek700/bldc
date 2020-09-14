#ifndef BUZZER_SETTINGS_H_
#define BUZZER_SETTINGS_H_

//List off all available sounds, each enum will star at prefix BUZZER_SOUND_xxx
#define BUZZER_SOUNDS_LIST(sound) 						\
	sound(START, 		 buzzer_sound_start)			\
	sound(SINGLE_PEAK, 	 buzzer_sound_single_peak)		\
	sound(DOUBLE_PEAK, 	 buzzer_sound_double_peak)		\
	sound(TRIPLE_PEAK, 	 buzzer_sound_triple_peak)		\
	sound(TURN_OFF, 	 buzzer_sound_turn_off)			\
	sound(DOUBLE_RISING, buzzer_sound_double_rising)

//Define function which generate a frequency
#define BUZZER_GENERATE_SOUND(freq)					buzzer_timer_set_frequency(freq)

#endif
