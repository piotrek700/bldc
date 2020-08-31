#ifndef FRAME_FRAMES_H_
#define FRAME_FRAMES_H_

#include "../Rybos/rybos.h"

//TODO implement load size
#if RADIO_MASTER == 1
#define FRAME_SYSTEM_LOAD_LENGTH_MASTER	(RYBOS_TASK_AND_IRQ_SIZE)
#define FRAME_SYSTEM_LOAD_LENGTH_SLAVE	(20+1)
#else
#define FRAME_SYSTEM_LOAD_LENGTH_MASTER	(20+1)
#define FRAME_SYSTEM_LOAD_LENGTH_SLAVE	(RYBOS_TASK_AND_IRQ_SIZE)
#endif

//Display types
#define FRAME_MAX_DISPLAY_CHANNELS_8	14

//Type and parameters mask
#define FRAME_TYPE_MASK					0x3F
#define FRAME_PARAM_MASK				0xC0

//Frame start symbol
#define FRAME_START_SYMBOL				0xA5

//Define all variables required to protocol
//1 parameter - enum generation - REQ_INIT_DATA -> FRAME_TYPE_REQ_INIT_DATA
//2 parameter - frame type name
//3 parameter - callback generation - req_init_data -> frame_cb_req_init_data
//Warning! - size of any type must be equal or lower then 255
//Warning! - define all structures by using uint_xt or int_xt types
#define FRAME_DICTIONARY_DEFINITION(row)                                                 \
		row(REQ_INIT_DATA,           FrameReqInitData,          req_init_data		 	)\
		row(RESP_INIT_DATA, 		 FrameRespInitData,         resp_init_data 		 	)\
		row(ERROR_LOG,               FrameErrorLog,             error_log			  	)\
		row(SLOW_PARAMS_SLAVE,       FrameSlowParamSlave,       slow_param_slave        )\
		row(SLOW_PARAMS_MASTER,      FrameSlowParamMaster,      slow_param_master	  	)\
		row(FAST_PARAMS_SLAVE,       FrameFastParamsSlave,      fast_param_slave       	)\
		row(FAST_PARAMS_MASTER, 	 FrameFastParamsMaster,     fast_param_master	  	)\
		row(RADIO_STAT, 			 FrameRadioStat,            radio_stat			 	)\
		row(SYSTEM_LOAD_SLAVE,       FrameSystemLoadSlave,      system_load_slave	  	)\
		row(SYSTEM_LOAD_MASTER, 	 FrameSystemLoadMaster,     system_load_master 	 	)\
		row(SYSTEM_PARAMS_SLAVE, 	 FrameSystemParamSlave,     system_param_slave	  	)\
		row(SYSTEM_PARAMS_MASTER,	 FrameSystemParamMaster,    system_param_master     )\
		row(RC_CONTROL, 			 FrameRcControl,            rc_control 			 	)\
		row(GET_PID_SETTINGS,		 FrameGetPidSettings,       get_pid_settings        )\
		row(SET_PID_SETTINGS, 		 FrameSetPidSettings,       set_pid_settings        )\
		row(REQ_DISPLAY_CHANNELS, 	 FrameReqDisplayChannels,   req_display_channels    )\
		row(RESP_DISPLAY_CHANNELS,   FrameRespDisplayChannels,  resp_display_channels   )\
		row(DISPLAY_CHANNELS_DATA_2, FrameDisplayChannelsData2, display_channels_data_2 )\
		row(DISPLAY_CHANNELS_DATA_4, FrameDisplayChannelsData4, display_channels_data_4 )\
		row(DISPLAY_CHANNELS_DATA_8, FrameDisplayChannelsData8, display_channels_data_8 )\
		row(UART_STAT,               FrameUartStat,             uart_stat               )

//Frame status masks
typedef enum {
	FRAME_STATUS_BUTTON_LT 			= (1<<0),
	FRAME_STATUS_BUTTON_RT 			= (1<<1),
	FRAME_STATUS_BUTTON_LB 			= (1<<2),
	FRAME_STATUS_BUTTON_RB 			= (1<<3),
	FRAME_STATUS_BUTTON_JOYL 		= (1<<4),
	FRAME_STATUS_BUTTON_JOYR 		= (1<<5),
	FRAME_STATUS_USB_PRESENT 		= (1<<6),
	FRAME_STATUS_LOCK 				= (1<<7),
	FRAME_STATUS_SLAVE_CONNECTED 	= (1<<8),
	FRAME_STATUS_CHARGING 			= (1<<9),
	FRAME_STATUS_NO_RC_BATTERY 		= (1<<10),

	FRAME_STATUS_BUTTON_MASK 		= 0x0038
} FrameStatus;

//Parameters
typedef enum{
	//Source
	FRAME_SOURCE_MASTER				= 0x80,
	FRAME_SOURCE_SLAVE				= 0x00,
	//Destination
	FRAME_DESTINATION_MASTER_PC		= 0x40,
	FRAME_DESTINATION_SLAVE			= 0x80
}FrameParams;

//PID type
typedef enum{
	FRAME_PID_TYPE_PITCH_ROLL,
	FRAME_PID_TYPE_YAW,
	FRAME_PID_TYPE_HEIGHT
}FramePidType;

typedef enum {
	FRAME_DISPLAY_CHANNEL_I1,
	FRAME_DISPLAY_CHANNEL_I2,
	FRAME_DISPLAY_CHANNEL_I3,
}FrameDisplayChannels;

typedef enum {
	FRAME_DISPLAY_MODE_2_CHANNELS,
	FRAME_DISPLAY_MODE_4_CHANNELS,
	FRAME_DISPLAY_MODE_8_CHANNELS,
}FrameDisplayMode;

//Frame structures definition
//0
typedef struct __attribute__((__packed__)){
	uint8_t none;				//TODO remove
}FrameReqInitData;

//1
typedef struct __attribute__((__packed__)){
	uint32_t magic_init_number;
	uint32_t uuid[3];
	uint8_t compilation_time[8];
	uint8_t compilation_date[11];
}FrameRespInitData;

//2
typedef struct __attribute__((__packed__)){
	uint8_t error;
}FrameErrorLog;

//3
typedef struct __attribute__((__packed__)){
	uint16_t ntc_temp;									//	-128 	- 	256		C
	uint16_t up_temp;									//	-128 	- 	256		C
	uint16_t bat_v;										//	   0 	-  	 32 	V
	uint16_t ldo_v;										//	   0 	-     4		V
}ParamAdcSlave;

typedef struct __attribute__((__packed__)){
	uint16_t temp;										//	-128 	- 	256		C
	int16_t	acc[3];										//	 -80 	-    80		m/s2
	int16_t vel[3];										//   -2k 	-  	 2k		deg/s
}ParamImu;

typedef struct __attribute__((__packed__)){
	uint16_t press;										//	 90k	-  110k 	Pa
	uint16_t temp;										//	-128 	- 	256		C
	int16_t height;										// -1000 	-  1000		m
}ParamPressure;

typedef struct __attribute__((__packed__)){
	ParamAdcSlave adc;									// 	  8B
	ParamImu imu;										//	 14B
	ParamPressure pressure;								// 	  6B
}FrameSlowParamSlave;									//	 1/s	28B/s		224b/s

//4
typedef struct __attribute__((__packed__)){
	uint16_t ntc_bat_temp;								//	-128 	- 	256		C
	uint16_t up_temp;									//	-128 	- 	256		C
	uint16_t bat_v;										//	   0 	-  	  8 	V
	uint16_t ldo_v;										//	   0 	-     4		V
	uint16_t usb_v;										//	   0 	-     8		V
}ParamAdcMaster;

typedef struct __attribute__((__packed__)){
	ParamAdcMaster adc;									// 	12B
	ParamImu imu;										// 	14B
	ParamPressure pressure;								// 	6B
	uint16_t disp_brighntess;
}FrameSlowParamMaster;									//	25/s 	 800B/s		6400b/s

//5
typedef struct __attribute__((__packed__)){
	//[0w][1x][2y][3z]
	int16_t	q[4];										//	  -1 	-     1
}ParamAhrs;

typedef struct __attribute__((__packed__)){
	//[0L][1T][2R][3L]
	int16_t	angle[4];									//	-180 	-  	180		deg
}ParamServo;

typedef struct __attribute__((__packed__)){
	ParamAhrs ahrs;										//	  8B
	ParamServo servo;									//	  8B
}FrameFastParamsSlave;									//	25/s 	 400B/s		3200b/s

//6
typedef struct __attribute__((__packed__)){
	int16_t joy_l_v;									//	-2048	-	+2047
	int16_t joy_l_h;									//	-2048	-	+2047
	int16_t joy_r_v;									//	-2048	-	+2047
	int16_t joy_r_h;									//	-2048	-	+2047
	uint16_t pot_l;										//		0	-	4095
	uint16_t pot_r;										//		0	-	4095
}ParamPotentiometers;

typedef struct __attribute__((__packed__)){
	ParamAhrs ahrs;										//	8B
	ParamPotentiometers pot;							//	12B
}FrameFastParamsMaster;

//7
typedef struct __attribute__((__packed__)){
	uint16_t tx_bytes;									//	   0	- 65534
	uint16_t rx_bytes;									//	   0	- 65534
	uint16_t rettransmition;							//	   0	- 65534
	uint16_t avarage_rssi;								//  -256	-	 32		db
	uint16_t max_rssi;									//  -256	-	 32		db
	uint16_t min_rssi;									//	-256	-	 32		db
	uint8_t	max_tran_deph;								//	   0	-	255
	uint16_t recived_frame;								//	   0	- 65534
	uint16_t recived_frame_total;						//    0    - 0xFFFFFFFF
	uint16_t transmitted_frame;							//	   0	- 65534
}FrameRadioStat;										//	 13B
														//  1/5s		 2.6B/s		20.8b/s
//8
typedef struct __attribute__((__packed__)){
	uint16_t load[FRAME_SYSTEM_LOAD_LENGTH_SLAVE];		//	   0	-	100		%
}FrameSystemLoadSlave;									//  1/5s		6,8B/s		54.4b/s

//9
typedef struct __attribute__((__packed__)){
	uint16_t load[FRAME_SYSTEM_LOAD_LENGTH_MASTER];		//	   0	-	100		%
}FrameSystemLoadMaster;									//  1/5s		6,8B/s		54.4b/s

//10
typedef struct __attribute__((__packed__)){
	uint32_t system_local_time;							//	   0	-  0xFF
	uint8_t critical_deh;								//     0	-	255
	uint8_t spi_tran_deph;								//	   0	-	255
}FrameSystemParamSlave;									//  1/5s		1.2B/s		9.6b/s

//11
typedef struct __attribute__((__packed__)){
	uint32_t system_local_time;							//	   0	-  0xFF
	uint8_t critical_deh;								//     0	-	255
	uint8_t spi_tran_deph;								//	   0	-	255
	uint8_t display_tran_depth;							//	   0 	-	255
}FrameSystemParamMaster;								//  1/5s		1.2B/s		9.6b/s

//12
typedef struct __attribute__((__packed__)){
	int16_t yaw;										//	-2048	-	+2047
	int16_t pitch;										//	-2048	-	+2047
	int16_t roll;										//	-2048	-	+2047
	int16_t throttle;									//	-2048	-	+2047
	uint16_t status;
}FrameRcControl;

//13
typedef struct __attribute__((__packed__)){
	uint8_t pid_type;
}FrameGetPidSettings;

//14
typedef struct __attribute__((__packed__)){
	uint8_t pid_type;
	float kp;
	float ki;
	float kd;
	float i_max;
}FrameSetPidSettings;

//15
typedef struct __attribute__((__packed__)){
	uint8_t none;										//TODO remove
}FrameReqDisplayChannels;

//16
typedef struct __attribute__((__packed__)){
	uint8_t channels[8];
	uint8_t mode;
}FrameRespDisplayChannels;

//17
typedef struct __attribute__((__packed__)){
	int16_t ch1[FRAME_MAX_DISPLAY_CHANNELS_8 * 4];
	int16_t ch2[FRAME_MAX_DISPLAY_CHANNELS_8 * 4];
	uint8_t packet_cnt;
}FrameDisplayChannelsData2;

//18
typedef struct __attribute__((__packed__)){
	int16_t ch1[FRAME_MAX_DISPLAY_CHANNELS_8 * 2];
	int16_t ch2[FRAME_MAX_DISPLAY_CHANNELS_8 * 2];
	int16_t ch3[FRAME_MAX_DISPLAY_CHANNELS_8 * 2];
	int16_t ch4[FRAME_MAX_DISPLAY_CHANNELS_8 * 2];
	uint8_t packet_cnt;
}FrameDisplayChannelsData4;

//19
typedef struct __attribute__((__packed__)){
	int16_t ch1[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch2[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch3[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch4[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch5[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch6[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch7[FRAME_MAX_DISPLAY_CHANNELS_8];
	int16_t ch8[FRAME_MAX_DISPLAY_CHANNELS_8];
	uint8_t packet_cnt;
}FrameDisplayChannelsData8;

//20
typedef struct __attribute__((__packed__)){
	uint32_t tx_bytes;									//	   0	- 65534
	uint32_t rx_bytes;									//	   0	- 65534
	uint16_t rx_error_frames;							//	   0	- 65534
	uint16_t recived_frames;							//	   0	- 65534
	uint16_t transmitted_frames;						//	   0	- 65534
	uint8_t	max_tran_deph;								//	   0	-	255
}FrameUartStat;

#endif
