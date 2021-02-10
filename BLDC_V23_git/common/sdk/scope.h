#ifndef SCOPE_H_
#define SCOPE_H_

#include <sdk/frame.h>

#define SCOPE_FRAME_8CH_BUFF_SIZE		8

typedef union {
	FrameDisplayChannelsData2_t channels2[SCOPE_FRAME_8CH_BUFF_SIZE * 4];
	FrameDisplayChannelsData4_t channels4[SCOPE_FRAME_8CH_BUFF_SIZE * 2];
	FrameDisplayChannelsData8_t channels8[SCOPE_FRAME_8CH_BUFF_SIZE * 1];
} ScopeFrameBuffer_t;

typedef enum{
	SCOPE_MODE_NOT_SET,
	SCOPE_2CH_MODE,
	SCOPE_4CH_MODE,
	SCOPE_8CH_MODE,
} ScopeMode_t;

uint32_t scope_get_max_queue_depth(void);

FrameDisplayChannelsData2_t * scope_get_2ch_frame(uint32_t index);

FrameDisplayChannelsData4_t * scope_get_4ch_frame(uint32_t index);

FrameDisplayChannelsData8_t * scope_get_8ch_frame(uint32_t index);

void scope_send_2ch(int16_t ch1, int16_t ch2);

void scope_send_4ch(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4);

void scope_send_8ch(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4, int16_t ch5, int16_t ch6, int16_t ch7, int16_t ch8);

void scope_state_machine(void);

#endif
