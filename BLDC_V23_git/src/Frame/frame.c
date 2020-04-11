#include "frame.h"
#include "frame_frames.h"
#include "../Debug/debug.h"
#include "../Uart/uart.h"
#include "../Radio/radio.h"
#include "../utils.h"

static volatile uint8_t frame_buffer[FRAME_BUFFER_LENGTH + 1];	//TODO +2 not +1 = frame type + crc - add some buffer -mayby it will be good to have hera always 255
static volatile FrameType frame_type;
static volatile uint8_t frame_params;

static volatile uint32_t payload_ptr = 0;
static volatile uint8_t crc_rx = 0;
static volatile FrameDecoderStatus decoder_status = WAIT_FOR_START;

void __attribute__((weak)) frame_cb_frame_req_init_data(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameReqInitData *frame = (FrameReqInitData *) buff;
}

void __attribute__((weak)) frame_cb_frame_resp_init_data(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameRespInitData *frame = (FrameRespInitData *) buff;
}

void __attribute__((weak)) frame_cb_frame_error_log(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameErrorLog *frame = (FrameErrorLog *) buff;
}

void __attribute__((weak)) frame_cb_frame_slow_param_slave(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSlowParamSlave *frame = (FrameSlowParamSlave *) buff;
}

void __attribute__((weak)) frame_cb_frame_slow_param_master(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSlowParamMaster *frame = (FrameSlowParamMaster *) buff;
}

void __attribute__((weak)) frame_cb_frame_fast_param_slave(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameFastParamsSlave *frame = (FrameFastParamsSlave *) buff;
}

void __attribute__((weak)) frame_cb_frame_fast_param_master(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameFastParamsMaster *frame = (FrameFastParamsMaster *) buff;
}

void __attribute__((weak)) frame_cb_frame_radio_stat(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameRadioStat *frame = (FrameRadioStat *) buff;
}

void __attribute__((weak)) frame_cb_frame_system_load_slave(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSystemLoadSlave *frame = (FrameSystemLoadSlave *) buff;
}

void __attribute__((weak)) frame_cb_frame_system_load_master(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSystemLoadMaster *frame = (FrameSystemLoadMaster *) buff;
}

void __attribute__((weak)) frame_cb_frame_system_param_slave(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSystemParamSlave *frame = (FrameSystemParamSlave *) buff;
}

void __attribute__((weak)) frame_cb_frame_system_param_master(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSystemParamMaster *frame = (FrameSystemParamMaster *) buff;
}

void __attribute__((weak)) frame_cb_frame_rc_control(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameRcControl *frame = (FrameRcControl *) buff;
}

void __attribute__((weak)) frame_cb_frame_get_pid_settings(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameGetPidSettings *frame = (FrameGetPidSettings *) buff;
}

void __attribute__((weak)) frame_cb_frame_set_pid_settings(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameSetPidSettings *frame = (FrameSetPidSettings *) buff;
}

void __attribute__((weak)) frame_cb_frame_req_display_channels(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameReqDisplayChannels *frame = (FrameReqDisplayChannels *) buff;
}

void __attribute__((weak)) frame_cb_frame_resp_display_channels(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameRespDisplayChannels *frame = (FrameRespDisplayChannels *) buff;
}

void __attribute__((weak)) frame_cb_frame_display_channels_data_2(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameDisplayChannelsData2 *frame = (FrameDisplayChannelsData2 *) buff;
}

void __attribute__((weak)) frame_cb_frame_display_channels_data_4(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameDisplayChannelsData4 *frame = (FrameDisplayChannelsData4 *) buff;
}

void __attribute__((weak)) frame_cb_frame_display_channels_data_8(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameDisplayChannelsData8 *frame = (FrameDisplayChannelsData8 *) buff;
}

void __attribute__((weak)) frame_cb_frame_uart_stat(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	//TODO FrameUartStat *frame = (FrameUartStat *) buff;
}

static const FrameDictonary frame_dictionary[] = {
		{ frame_cb_frame_req_init_data, 			FRAME_TYPE_REQ_INIT_DATA, 				sizeof(FrameReqInitData) },
		{ frame_cb_frame_resp_init_data, 			FRAME_TYPE_RESP_INIT_DATA, 				sizeof(FrameRespInitData) },
		{ frame_cb_frame_error_log, 				FRAME_TYPE_ERROR_LOG, 					sizeof(FrameErrorLog) },
		{ frame_cb_frame_slow_param_slave, 			FRAME_TYPE_SLOW_PARAMS_SLAVE, 			sizeof(FrameSlowParamSlave) },
		{ frame_cb_frame_slow_param_master, 		FRAME_TYPE_SLOW_PARAMS_MASTER, 			sizeof(FrameSlowParamMaster) },
		{ frame_cb_frame_fast_param_slave, 			FRAME_TYPE_FAST_PARAMS_SLAVE, 			sizeof(FrameFastParamsSlave) },
		{ frame_cb_frame_fast_param_master, 		FRAME_TYPE_FAST_PARAMS_MASTER, 			sizeof(FrameFastParamsMaster) },
		{ frame_cb_frame_radio_stat, 				FRAME_TYPE_RADIO_STAT, 					sizeof(FrameRadioStat) },
		{ frame_cb_frame_system_load_slave, 		FRAME_TYPE_SYSTEM_LOAD_SLAVE, 			sizeof(FrameSystemLoadSlave) },
		{ frame_cb_frame_system_load_master, 		FRAME_TYPE_SYSTEM_LOAD_MASTER, 			sizeof(FrameSystemLoadMaster) },
		{ frame_cb_frame_system_param_slave,		FRAME_TYPE_SYSTEM_PARAMS_SLAVE, 		sizeof(FrameSystemParamSlave) },
		{ frame_cb_frame_system_param_master, 		FRAME_TYPE_SYSTEM_PARAMS_MASTER,		sizeof(FrameSystemParamMaster) },
		{ frame_cb_frame_rc_control, 				FRAME_TYPE_RC_CONTROL, 					sizeof(FrameRcControl) },
		{ frame_cb_frame_get_pid_settings, 			FRAME_TYPE_GET_PID_SETTINGS, 			sizeof(FrameGetPidSettings) },
		{ frame_cb_frame_set_pid_settings, 			FRAME_TYPE_SET_PID_SETTINGS, 			sizeof(FrameSetPidSettings) },
		{ frame_cb_frame_req_display_channels, 		FRAME_TYPE_REQ_DISPLAY_CHANNELS, 		sizeof(FrameReqDisplayChannels) },
		{ frame_cb_frame_resp_display_channels, 	FRAME_TYPE_RESP_DISPLAY_CHANNELS, 		sizeof(FrameRespDisplayChannels) },
		{ frame_cb_frame_display_channels_data_2, 	FRAME_TYPE_DISPLAY_CHANNELS_DATA_2, 	sizeof(FrameDisplayChannelsData2) },
		{ frame_cb_frame_display_channels_data_4, 	FRAME_TYPE_DISPLAY_CHANNELS_DATA_4, 	sizeof(FrameDisplayChannelsData4) },
		{ frame_cb_frame_display_channels_data_8, 	FRAME_TYPE_DISPLAY_CHANNELS_DATA_8, 	sizeof(FrameDisplayChannelsData8) },
		{ frame_cb_frame_uart_stat, 				FRAME_TYPE_UART_STAT, 					sizeof(FrameUartStat) },
};

static void frame_check_decoding(void) {
	static uint8_t payload_length = 0;
	static void (*cb_handler)(void *, uint8_t) = 0;

	//Get frame type
	if (payload_ptr == 1) {
		frame_type = frame_buffer[payload_ptr - 1] & FRAME_TYPE_MASK;
		frame_params = frame_buffer[payload_ptr - 1] & FRAME_PARAM_MASK;

		//Check frame type and get length
		if (frame_type >= sizeof(frame_dictionary) / sizeof(FrameDictonary)) {
			debug_error(FRAME_TYPE_NOT_SUPPORTED);
			payload_ptr = 0;
			decoder_status = WAIT_FOR_START;
			return;
		}

		payload_length = frame_dictionary[frame_type].frame_size + 2; //+ frame type +crc
		cb_handler = frame_dictionary[frame_type].cb;
		return;
	}

	//Buffer overflow protection
	if (payload_ptr > FRAME_BUFFER_LENGTH) {
		debug_error(FRAME_BUFFER_OVERFLOW);
		payload_ptr = 0;
		decoder_status = WAIT_FOR_START;
		return;
	}

	//Check receiving complete
	if (payload_ptr == payload_length) {
		if(decoder_status == START_DETECTED){
			crc_rx-=frame_buffer[payload_ptr - 1];
		}

		//Check CRC
		if (frame_buffer[payload_ptr - 1] == crc_rx) {
			uart_increment_reveived_frame_cnt();

			if(frame_params & FRAME_DESTINATION_MASTER_PC){
				//PC->Slave
				cb_handler((void *) (frame_buffer + 1), frame_params);
			}else{
				//PC->Master->Slave
				frame_radio_send(frame_type, (uint8_t *) (frame_buffer + 1), frame_params);
			}
		} else {
			uart_increment_received_error_frame_cnt();
			debug_error(FRAME_CRC_ERROR);
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

void frame_call_received_cb(FrameType frame_type, uint8_t *frame, uint8_t params) {
	frame_dictionary[frame_type].cb((void *) frame, params);
}

void frame_uart_send(FrameType frame_type, uint8_t *frame, uint32_t params) {
	uart_send(frame_type | params, frame, frame_dictionary[frame_type].frame_size);
}

void frame_radio_send(FrameType frame_type, uint8_t *frame, uint32_t params) {
	radio_send(frame_type | params, frame, frame_dictionary[frame_type].frame_size);

	frame_uart_send(frame_type, frame, params);
}

void frame_test(void) {
	//TODO Test
}
