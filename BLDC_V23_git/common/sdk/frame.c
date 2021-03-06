#include <sdk/frame.h>
#include <sdk/debug.h>
#include <sdk/utils.h>

static uint8_t frame_buffer[sizeof(FrameRxBuffer_t) + 1];	//frame type + crc
static FrameType_t frame_type;
static FrameParams_t frame_params;

static uint32_t payload_ptr = 0;
static uint8_t crc_rx = 0;
static FrameDecoderStatus_t decoder_status = WAIT_FOR_START;

//Generate all weak definitions
FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_WEAK_DECLARATION);

//Define dictionary
static const FrameDictonary_t frame_dictionary[] = {
		FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_DITIONARY)
};

//Frame received complete
void __attribute__((weak)) frame_received_complete(FrameType_t type, FrameParams_t params, uint8_t *buff, FrameCb_t cb) {
	//Call original frame callback
	cb((void *) (buff), params);

	UNUSED(type);
}

//Frame received error
void __attribute__((weak)) frame_received_error(void) {
	debug_error(FRAME_CRC_ERROR);
}

static void frame_check_decoding(void) {
	static uint8_t payload_length = 0;
	static FrameCb_t cb_handler = 0;

	//Get frame type
	if (payload_ptr == 1) {
		frame_type = frame_buffer[payload_ptr - 1] & FRAME_TYPE_MASK;
		frame_params = frame_buffer[payload_ptr - 1] & FRAME_PARAM_MASK;

		//Check frame type and get length
		if (frame_type >= sizeof(frame_dictionary) / sizeof(FrameDictonary_t)) {
			debug_error(FRAME_TYPE_NOT_SUPPORTED);
			payload_ptr = 0;
			decoder_status = WAIT_FOR_START;
			return;
		}

		payload_length = frame_dictionary[frame_type].frame_size + 2; //+ frame type +crc
		cb_handler = frame_dictionary[frame_type].callback;
		return;
	}

	//Buffer overflow protection
	if (payload_ptr > sizeof(FrameRxBuffer_t)) {
		debug_error(FRAME_BUFFER_OVERFLOW);
		payload_ptr = 0;
		decoder_status = WAIT_FOR_START;
		return;
	}

	//Check receiving complete
	if (payload_ptr == payload_length) {
		if (decoder_status == START_DETECTED) {
			crc_rx -= frame_buffer[payload_ptr - 1];
		}

		//Check CRC
		if (frame_buffer[payload_ptr - 1] == crc_rx) {
			frame_received_complete(frame_type, frame_params, frame_buffer + 1, cb_handler);
		} else {
			frame_received_error();
		}

		payload_ptr = 0;
		decoder_status = WAIT_FOR_START;
	}
}

void frame_decoding_state_mashine(uint8_t data) {
	switch (decoder_status) {
	case WAIT_FOR_START:
		if (data == FRAME_START_SYMBOL) {
			decoder_status = START_DETECTED;
		}
		break;

	case START_DETECTED:
		if (data == FRAME_START_SYMBOL) {
			crc_rx += data;
		} else {
			payload_ptr = 0;
			crc_rx = data;
		}
		frame_buffer[payload_ptr++] = data;
		frame_check_decoding();
		decoder_status = DATA_LOADING;
		break;

	case DATA_LOADING:
		if (data == FRAME_START_SYMBOL) {
			decoder_status = START_DETECTED;
		} else {
			frame_buffer[payload_ptr++] = data;

			frame_check_decoding();
			crc_rx += data;
		}
		break;

	default:
		debug_error(FRAME_DECODING_STATE_MASHINE_NO_STATE);
		break;
	}
}

void frame_call_received_cb(FrameType_t type, FrameParams_t params, uint8_t *p_buff) {
	frame_dictionary[type].callback((void *) p_buff, params);
}

uint32_t frame_get_type_length(FrameType_t type) {
	return frame_dictionary[type].frame_size;
}

CCMRAM_FUCNTION static inline bool frame_code_symbol(uint8_t source, uint8_t *p_dest, uint32_t *p_len, uint8_t *p_crc, uint32_t dest_size_max) {
	if (source == FRAME_START_SYMBOL) {
		*(p_dest + *p_len) = source;
		(*p_len)++;

		if ((*p_len) >= dest_size_max) {
			debug_error(FRAME_TX_BUFFER_OVERFLOW);
			return false;
		}
	}
	*(p_dest + *p_len) = source;
	(*p_len)++;
	*p_crc += (uint8_t) source;

	if ((*p_len) >= dest_size_max) {
		debug_error(FRAME_TX_BUFFER_OVERFLOW);
		return false;
	}
	return true;
}

CCMRAM_FUCNTION uint32_t frame_send_coded(FrameType_t type, FrameParams_t params, uint8_t *p_source, uint8_t *p_dest, uint32_t dest_size_max) {
	uint32_t len = 0;
	uint8_t crc_tmp = 0;

	//Start symbol
	p_dest[len] = FRAME_START_SYMBOL;
	len++;

	//Type
	if (!frame_code_symbol(type | params, p_dest, &len, &crc_tmp, dest_size_max)) {
		return 0;
	}

	//Payload
	uint32_t i;
	for (i = 0; i < frame_dictionary[type].frame_size; i++) {
		if (!frame_code_symbol(p_source[i], p_dest, &len, &crc_tmp, dest_size_max)) {
			return 0;
		}
	}

	//CRC
	if (!frame_code_symbol(crc_tmp, p_dest, &len, &crc_tmp, dest_size_max)) {
		return 0;
	}

	//Return length
	return len;
}
