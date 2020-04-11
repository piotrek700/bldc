#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>

//Frame parameters
#define FRAME_START_SYMBOL				0xA5
#define FRAME_BUFFER_LENGTH				256		//TODO check

//Source
#define FRAME_SOURCE_MASTER				0x80
#define FRAME_SOURCE_SLAVE				0x00

//Destination
#define FRAME_DESTINATION_MASTER_PC		0x40
#define FRAME_DESTINATION_SLAVE			0x00

//Masks
#define FRAME_NONE_PARAM				0x00
#define FRAME_TYPE_MASK					0x3F
#define FRAME_PARAM_MASK				0xC0

//Frame status masks
#define FRAME_STATUS_BUTTON_LT			0x0001
#define FRAME_STATUS_BUTTON_RT			0x0002
#define FRAME_STATUS_BUTTON_LB			0x0004
#define FRAME_STATUS_BUTTON_RB			0x0008
#define FRAME_STATUS_BUTTON_JOYL		0x0010
#define FRAME_STATUS_BUTTON_JOYR		0x0020
#define FRAME_STATUS_USB_PRESENT		0x0040
#define FRAME_STATUS_LOCK				0x0080
#define FRAME_STATUS_SLAVE_CONNECTED	0x0100
#define FRAME_STATUS_CHARGING			0x0200
#define FRAME_STATUS_NO_RC_BATTERY		0x0400

#define FRAME_STATUS_BUTTON_MASK		0x0038

//PID type
#define FRAME_PID_PITCH_ROLL			0x00
#define FRAME_PID_YAW					0x01
#define FRAME_PID_HEIGHT				0x02

//Decoder status
typedef enum {
	DATA_LOADING,
	START_DETECTED,
	WAIT_FOR_START,
} FrameDecoderStatus;

// 1. Define new frame to frame type, first element uint8_t address
// 2. Create callback function
// 3. Add frame and callback to the dictionary in frame.c in the same order as in FrameType

//Frame types
typedef enum {
	FRAME_TYPE_REQ_INIT_DATA = 0, 			//0 	Do not remove that and do not reinit value, MAX 0x3f=63
	FRAME_TYPE_RESP_INIT_DATA,				//1
	FRAME_TYPE_ERROR_LOG,					//2
	FRAME_TYPE_SLOW_PARAMS_SLAVE,			//3
	FRAME_TYPE_SLOW_PARAMS_MASTER,			//4
	FRAME_TYPE_FAST_PARAMS_SLAVE,			//5
	FRAME_TYPE_FAST_PARAMS_MASTER,			//6
	FRAME_TYPE_RADIO_STAT,					//7
	FRAME_TYPE_SYSTEM_LOAD_SLAVE,			//8
	FRAME_TYPE_SYSTEM_LOAD_MASTER,			//9
	FRAME_TYPE_SYSTEM_PARAMS_SLAVE,			//10
	FRAME_TYPE_SYSTEM_PARAMS_MASTER,		//11
	FRAME_TYPE_RC_CONTROL,					//12
	FRAME_TYPE_GET_PID_SETTINGS,			//13
	FRAME_TYPE_SET_PID_SETTINGS,			//14
	FRAME_TYPE_REQ_DISPLAY_CHANNELS,		//15
	FRAME_TYPE_RESP_DISPLAY_CHANNELS,		//16
	FRAME_TYPE_DISPLAY_CHANNELS_DATA_2,		//17
	FRAME_TYPE_DISPLAY_CHANNELS_DATA_4,		//18
	FRAME_TYPE_DISPLAY_CHANNELS_DATA_8,		//19
	FRAME_TYPE_UART_STAT,					//20
}FrameType;

typedef struct {
	void (*cb)(void *, uint8_t);
	FrameType frame_type;
	uint32_t frame_size;
} FrameDictonary;

void frame_cb_frame_req_init_data(void *buff, uint8_t params);
void frame_cb_frame_resp_init_data(void *buff, uint8_t params);
void frame_cb_frame_error_log(void *buff, uint8_t params);
void frame_cb_frame_slow_param_slave(void *buff, uint8_t params);
void frame_cb_frame_slow_param_master(void *buff, uint8_t params);
void frame_cb_frame_fast_param_slave(void *buff, uint8_t params);
void frame_cb_frame_fast_param_master(void *buff, uint8_t params);
void frame_cb_frame_radio_stat(void *buff, uint8_t params);
void frame_cb_frame_system_load_slave(void *buff, uint8_t params);
void frame_cb_frame_system_load_master(void *buff, uint8_t params);
void frame_cb_frame_system_param_slave(void *buff, uint8_t params);
void frame_cb_frame_system_param_master(void *buff, uint8_t params);
void frame_cb_frame_rc_control(void *buff, uint8_t params);
void frame_cb_frame_get_pid_settings(void *buff, uint8_t params);
void frame_cb_frame_set_pid_settings(void *buff, uint8_t params);
void frame_cb_frame_req_display_channels(void *buff, uint8_t params);
void frame_cb_frame_resp_display_channels(void *buff, uint8_t params);
void frame_cb_frame_display_channels_data_2(void *buff, uint8_t params);
void frame_cb_frame_display_channels_data_4(void *buff, uint8_t params);
void frame_cb_frame_display_channels_data_8(void *buff, uint8_t params);
void frame_cb_frame_uart_stat(void *buff, uint8_t params);

void frame_decoding_state_mashine(uint8_t data);

void frame_call_received_cb(FrameType frame_type, uint8_t *frame, uint8_t params);

void frame_uart_send(FrameType frame_type, uint8_t *frame, uint32_t params);

void frame_radio_send(FrameType frame_type, uint8_t *frame, uint32_t params);

void frame_test(void);

#endif
