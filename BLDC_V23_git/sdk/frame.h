#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>
#include <settings/frame_frames.h>

#define FRAME_GENERATE_UNION(type, struct, cb)										struct cb;
#define FRAME_GENERATE_FRAME_TYPE(type, struct, cb)									FRAME_TYPE_##type,
#define FRAME_GENERATE_DITIONARY(type, struct, cb)									{frame_cb_##cb, FRAME_TYPE_##type, sizeof(struct)},
#define FRAME_GENERATE_DEFINITION(type, struct, cb)									void frame_cb_##cb(void *buff, uint8_t params);

//Frame callback function prototype
#define FRAME_GENERATE_WEAK_DECLARATION(type, struct, cb)						\
		void __attribute__((weak)) frame_cb_##cb (void *buff, uint8_t params) {	\
			UNUSED(buff);														\
			UNUSED(params);														\
		}

//Frame general structure
typedef struct __attribute__((__packed__)) {
	uint8_t type_param;
	union{
		FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_UNION);
	}payload;
}FrameRxBuffer;

//Frame types enums
typedef enum{
	FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_FRAME_TYPE)
}FrameType;

//Decoder status
typedef enum {
	DATA_LOADING,
	START_DETECTED,
	WAIT_FOR_START,
} FrameDecoderStatus;

//Frame callback type
typedef void (*FrameCb)(void *, uint8_t);

//Frame dictionary row
typedef struct {
	FrameCb callback;
	FrameType frame_type;
	uint32_t frame_size;
} FrameDictonary;

//Generate all declarations
FRAME_DICTIONARY_DEFINITION(FRAME_GENERATE_DEFINITION);

void frame_received_complete(FrameType type, FrameParams params, uint8_t *buff, FrameCb cb);

void frame_received_error(void);

void frame_decoding_state_mashine(uint8_t data);

void frame_call_received_cb(FrameType type, FrameParams params, uint8_t *buff);

uint32_t frame_get_type_length(FrameType type);

uint32_t frame_send_coded(FrameType type, FrameParams params, uint8_t *source, uint8_t *dest, uint32_t dest_size_max);

void frame_test(void);

#endif
