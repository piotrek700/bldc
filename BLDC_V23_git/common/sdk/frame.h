#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>
#include <settings/frame_frames.h>

#define FRAME_GENERATE_UNION(type, structure, cb)									structure cb;
#define FRAME_GENERATE_FRAME_TYPE(type, structure, cb)								FRAME_TYPE_##type,
#define FRAME_GENERATE_DITIONARY(type, structure, cb)								{frame_cb_##cb, FRAME_TYPE_##type, sizeof(structure)},
#define FRAME_GENERATE_DEFINITION(type, structure, cb)								void frame_cb_##cb(void *buff, uint8_t params);

//Frame callback function prototype
#define FRAME_GENERATE_WEAK_DECLARATION(type, structure, cb)						\
		void __attribute__((weak)) frame_cb_##cb (void *p_buff, uint8_t params) {	\
			UNUSED(p_buff);															\
			UNUSED(params);															\
		}

//Frame general structure
typedef struct __attribute__((__packed__)) {
	uint8_t type_param;
	union{
		FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_UNION);
	}payload;
} FrameRxBuffer_t;

//Frame types enums
typedef enum{
	FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_FRAME_TYPE)
} FrameType_t;

//Decoder status
typedef enum {
	DATA_LOADING,
	START_DETECTED,
	WAIT_FOR_START,
} FrameDecoderStatus_t;

//Frame callback type
typedef void (*FrameCb_t)(void *, uint8_t);

//Frame dictionary row
typedef struct {
	FrameCb_t callback;
	FrameType_t frame_type;
	uint32_t frame_size;
} FrameDictonary_t;

//Generate all declarations
FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_DEFINITION);

void frame_received_complete(FrameType_t type, FrameParams_t params, uint8_t *p_buff, FrameCb_t cb);

void frame_received_error(void);

void frame_decoding_state_mashine(uint8_t data);

void frame_call_received_cb(FrameType_t type, FrameParams_t params, uint8_t *p_buff);

uint32_t frame_get_type_length(FrameType_t type);

uint32_t frame_send_coded(FrameType_t type, FrameParams_t params, uint8_t *p_source, uint8_t *p_dest, uint32_t dest_size_max);

#endif
