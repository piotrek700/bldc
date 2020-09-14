#include <scope.h>
#include <debug.h>
#include "Uart/uart.h"

//Scope
static volatile ScopeFrameBuffer scope_frame_buff;
static volatile uint32_t scope_write_ptr = 0;
static volatile uint32_t scope_read_ptr = 0;
static volatile uint32_t scope_frame_buff_elements = 0;
static volatile uint32_t scope_frame_buff_max_elements = 0;

static volatile ScopeMode scope_mode = SCOPE_MODE_NOT_SET;

static volatile uint32_t scope_frame_data_cnt = 0;
static volatile uint32_t scope_frame_packet_cnt = 0;

uint32_t scope_get_max_queue_depth(void){
	return scope_frame_buff_max_elements;
}

FrameDisplayChannelsData2 *scope_get_2ch_frame(uint32_t index) {
	return (FrameDisplayChannelsData2 *)(scope_frame_buff.channels2 + index);
}

FrameDisplayChannelsData4 *scope_get_4ch_frame(uint32_t index) {
	return (FrameDisplayChannelsData4 *)(scope_frame_buff.channels4 + index);
}

FrameDisplayChannelsData8 *scope_get_8ch_frame(uint32_t index) {
	return (FrameDisplayChannelsData8 *)(scope_frame_buff.channels8 + index);
}

void scope_send_2ch(int16_t ch1, int16_t ch2) {
	//Check scope mode
	if (scope_mode == SCOPE_MODE_NOT_SET) {
		scope_mode = SCOPE_2CH_MODE;
	} else if (scope_mode != SCOPE_2CH_MODE) {
		debug_error(SCOPE_MORE_THAN_ONE_MODE_USED);
	}

	//OVerflow detection
	if(scope_frame_buff_elements > FRAME_MAX_DISPLAY_CHANNELS_8 * 4){
		debug_error(SCOPE_BUFFER_OVERFLOW);
	}

	//Put data to the buffer
	scope_frame_buff.channels2[scope_write_ptr].ch1[scope_frame_data_cnt] = ch1;
	scope_frame_buff.channels2[scope_write_ptr].ch2[scope_frame_data_cnt] = ch2;

	//If frame array complete
	scope_frame_data_cnt++;
	if (scope_frame_data_cnt == FRAME_MAX_DISPLAY_CHANNELS_8 * 4) {
		scope_frame_buff.channels2[scope_write_ptr].packet_cnt = scope_frame_packet_cnt;
		scope_frame_packet_cnt++;
		scope_frame_data_cnt = 0;
		scope_frame_buff_elements++;

		if (scope_frame_buff_elements > scope_frame_buff_max_elements) {
			scope_frame_buff_max_elements = scope_frame_buff_elements;
		}

		scope_write_ptr++;
		if (scope_write_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 4) {
			scope_write_ptr = 0;
		}
	}
}

void scope_send_4ch(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4) {
	//Check scope mode
	if (scope_mode == SCOPE_MODE_NOT_SET) {
		scope_mode = SCOPE_4CH_MODE;
	} else if (scope_mode != SCOPE_4CH_MODE) {
		debug_error(SCOPE_MORE_THAN_ONE_MODE_USED);
	}

	//OVerflow detection
	if(scope_frame_buff_elements > FRAME_MAX_DISPLAY_CHANNELS_8 * 2){
		debug_error(SCOPE_BUFFER_OVERFLOW);
	}

	//Put data to the buffer
	scope_frame_buff.channels4[scope_write_ptr].ch1[scope_frame_data_cnt] = ch1;
	scope_frame_buff.channels4[scope_write_ptr].ch2[scope_frame_data_cnt] = ch2;
	scope_frame_buff.channels4[scope_write_ptr].ch3[scope_frame_data_cnt] = ch3;
	scope_frame_buff.channels4[scope_write_ptr].ch4[scope_frame_data_cnt] = ch4;

	//If frame array complete
	scope_frame_data_cnt++;
	if (scope_frame_data_cnt == FRAME_MAX_DISPLAY_CHANNELS_8 * 2) {
		scope_frame_buff.channels4[scope_write_ptr].packet_cnt = scope_frame_packet_cnt;
		scope_frame_packet_cnt++;
		scope_frame_data_cnt = 0;
		scope_frame_buff_elements++;

		if (scope_frame_buff_elements > scope_frame_buff_max_elements) {
			scope_frame_buff_max_elements = scope_frame_buff_elements;
		}

		scope_write_ptr++;
		if (scope_write_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 2) {
			scope_write_ptr = 0;
		}
	}
}

void scope_send_8ch(int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4, int16_t ch5, int16_t ch6, int16_t ch7, int16_t ch8) {
	//Check scope mode
	if (scope_mode == SCOPE_MODE_NOT_SET) {
		scope_mode = SCOPE_8CH_MODE;
	} else if (scope_mode != SCOPE_8CH_MODE) {
		debug_error(SCOPE_MORE_THAN_ONE_MODE_USED);
	}

	//OVerflow detection
	if(scope_frame_buff_elements > FRAME_MAX_DISPLAY_CHANNELS_8 * 1){
		debug_error(SCOPE_BUFFER_OVERFLOW);
	}

	//Put data to the buffer
	scope_frame_buff.channels8[scope_write_ptr].ch1[scope_frame_data_cnt] = ch1;
	scope_frame_buff.channels8[scope_write_ptr].ch2[scope_frame_data_cnt] = ch2;
	scope_frame_buff.channels8[scope_write_ptr].ch3[scope_frame_data_cnt] = ch3;
	scope_frame_buff.channels8[scope_write_ptr].ch4[scope_frame_data_cnt] = ch4;
	scope_frame_buff.channels8[scope_write_ptr].ch5[scope_frame_data_cnt] = ch5;
	scope_frame_buff.channels8[scope_write_ptr].ch6[scope_frame_data_cnt] = ch6;
	scope_frame_buff.channels8[scope_write_ptr].ch7[scope_frame_data_cnt] = ch7;
	scope_frame_buff.channels8[scope_write_ptr].ch8[scope_frame_data_cnt] = ch8;

	//If frame array complete
	scope_frame_data_cnt++;
	if (scope_frame_data_cnt == FRAME_MAX_DISPLAY_CHANNELS_8 * 1) {
		scope_frame_buff.channels8[scope_write_ptr].packet_cnt = scope_frame_packet_cnt;
		scope_frame_packet_cnt++;
		scope_frame_data_cnt = 0;
		scope_frame_buff_elements++;

		if (scope_frame_buff_elements > scope_frame_buff_max_elements) {
			scope_frame_buff_max_elements = scope_frame_buff_elements;
		}

		scope_write_ptr++;
		if (scope_write_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 1) {
			scope_write_ptr = 0;
		}
	}
}

void scope_state_machine(void) {
	while (scope_frame_buff_elements) {
		switch (scope_mode) {
		case SCOPE_2CH_MODE:
			uart_send_scope_frame(FRAME_TYPE_DISPLAY_CHANNELS_DATA_2, (uint8_t *) (scope_frame_buff.channels2 + scope_read_ptr), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
			scope_read_ptr++;

			if (scope_read_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 4) {
				scope_read_ptr = 0;
			}
			break;

		case SCOPE_4CH_MODE:
			uart_send_scope_frame(FRAME_TYPE_DISPLAY_CHANNELS_DATA_4, (uint8_t *) (scope_frame_buff.channels4 + scope_read_ptr), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
			scope_read_ptr++;

			if (scope_read_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 2) {
				scope_read_ptr = 0;
			}
			break;

		case SCOPE_8CH_MODE:
			uart_send_scope_frame(FRAME_TYPE_DISPLAY_CHANNELS_DATA_8, (uint8_t *) (scope_frame_buff.channels8 + scope_read_ptr), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
			scope_read_ptr++;

			if (scope_read_ptr == SCOPE_FRAME_8CH_BUFF_SIZE * 1) {
				scope_read_ptr = 0;
			}
			break;

		case SCOPE_MODE_NOT_SET:
			default:
			debug_error(SCOPE_SM_STATE_NOT_SUPPORTED);
			break;
		}

		scope_frame_buff_elements--;
	}
}
