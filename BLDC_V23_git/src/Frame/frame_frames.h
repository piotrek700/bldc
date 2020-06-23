#ifndef FRAME_FRAMES_H_
#define FRAME_FRAMES_H_

#include "../Rybos/rybos.h"
#include "../Radio/radio.h"

//TODO implement load size
#ifdef RADIO_MASTER
#define FRAME_SYSTEM_LOAD_LENGTH_MASTER	(MARKER_SYSTEM+1)
#define FRAME_SYSTEM_LOAD_LENGTH_SLAVE	(20+1)
#else
#define FRAME_SYSTEM_LOAD_LENGTH_MASTER	(20+1)
#define FRAME_SYSTEM_LOAD_LENGTH_SLAVE	(MARKER_SYSTEM+1)
#endif

//Display types
#define FRAME_MAX_DISPLAY_CHANNELS_8	14

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
	uint16_t ntc_temp;			//	-128 	- 	256		C
	uint16_t up_temp;			//	-128 	- 	256		C
	uint16_t bat_v;				//	   0 	-  	 32 	V
	uint16_t ldo_v;				//	   0 	-     4		V
}ParamAdcSlave;

typedef struct __attribute__((__packed__)){
	uint16_t temp;				//	-128 	- 	256		C
	int16_t	acc[3];				//	 -80 	-    80		m/s2
	int16_t vel[3];				//   -2k 	-  	 2k		deg/s
}ParamImu;

typedef struct __attribute__((__packed__)){
	uint16_t press;				//	 90k	-  110k 	Pa
	uint16_t temp;				//	-128 	- 	256		C
	int16_t height;				// -1000 	-  1000		m
}ParamPressure;

typedef struct __attribute__((__packed__)){
	ParamAdcSlave adc;			// 	  8B
	ParamImu imu;				//	 14B
	ParamPressure pressure;		// 	  6B
}FrameSlowParamSlave;			//	 1/s	28B/s		224b/s

//4
typedef struct __attribute__((__packed__)){
	uint16_t ntc_bat_temp;		//	-128 	- 	256		C
	uint16_t up_temp;			//	-128 	- 	256		C
	uint16_t bat_v;				//	   0 	-  	  8 	V
	uint16_t ldo_v;				//	   0 	-     4		V
	uint16_t usb_v;				//	   0 	-     8		V
}ParamAdcMaster;

typedef struct __attribute__((__packed__)){
	ParamAdcMaster adc;			// 	12B
	ParamImu imu;				// 	14B
	ParamPressure pressure;		// 	6B
	uint16_t disp_brighntess;
}FrameSlowParamMaster;			//	25/s 	 800B/s		6400b/s

//5
typedef struct __attribute__((__packed__)){
	//[0w][1x][2y][3z]
	int16_t	q[4];				//	  -1 	-     1
}ParamAhrs;

typedef struct __attribute__((__packed__)){
	//[0L][1T][2R][3L]
	int16_t	angle[4];			//	-180 	-  	180		deg
}ParamServo;

typedef struct __attribute__((__packed__)){
	ParamAhrs ahrs;				//	  8B
	ParamServo servo;			//	  8B
}FrameFastParamsSlave;			//	25/s 	 400B/s		3200b/s

//6
typedef struct __attribute__((__packed__)){
	int16_t joy_l_v;			//	-2048	-	+2047
	int16_t joy_l_h;			//	-2048	-	+2047
	int16_t joy_r_v;			//	-2048	-	+2047
	int16_t joy_r_h;			//	-2048	-	+2047
	uint16_t pot_l;				//		0	-	4095
	uint16_t pot_r;				//		0	-	4095
}ParamPotentiometers;

typedef struct __attribute__((__packed__)){
	ParamAhrs ahrs;				//	8B
	ParamPotentiometers pot;	//	12B
}FrameFastParamsMaster;

//7
typedef struct __attribute__((__packed__)){
	uint16_t tx_bytes;			//	   0	- 65534
	uint16_t rx_bytes;			//	   0	- 65534
	uint16_t rettransmition;	//	   0	- 65534
	uint16_t avarage_rssi;		//  -256	-	 32		db
	uint16_t max_rssi;			//  -256	-	 32		db
	uint16_t min_rssi;			//	-256	-	 32		db
	uint8_t	max_tran_deph;		//	   0	-	255
	uint16_t recived_frame;		//	   0	- 65534
	uint16_t recived_frame_total;//    0    - 0xFFFFFFFF
	uint16_t transmitted_frame;	//	   0	- 65534
}FrameRadioStat;				//	 13B
								//  1/5s		 2.6B/s		20.8b/s
//8
typedef struct __attribute__((__packed__)){
	uint16_t load[FRAME_SYSTEM_LOAD_LENGTH_SLAVE];			//	   0	-	100		%
}FrameSystemLoadSlave;			//  1/5s		6,8B/s		54.4b/s

//9
typedef struct __attribute__((__packed__)){
	uint16_t load[FRAME_SYSTEM_LOAD_LENGTH_MASTER];			//	   0	-	100		%
}FrameSystemLoadMaster;			//  1/5s		6,8B/s		54.4b/s

//10
typedef struct __attribute__((__packed__)){
	uint32_t system_local_time;	//	   0	-  0xFF
	uint8_t critical_deh;		//     0	-	255
	uint8_t spi_tran_deph;		//	   0	-	255
}FrameSystemParamSlave;				//  1/5s		1.2B/s		9.6b/s

//11
typedef struct __attribute__((__packed__)){
	uint32_t system_local_time;	//	   0	-  0xFF
	uint8_t critical_deh;		//     0	-	255
	uint8_t spi_tran_deph;		//	   0	-	255
	uint8_t display_tran_depth;		//	   0 	-	255
}FrameSystemParamMaster;				//  1/5s		1.2B/s		9.6b/s

//12
typedef struct __attribute__((__packed__)){
	int16_t yaw;				//	-2048	-	+2047
	int16_t pitch;				//	-2048	-	+2047
	int16_t roll;				//	-2048	-	+2047
	int16_t throttle;			//	-2048	-	+2047
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
	uint8_t none;				//TODO remove
}FrameReqDisplayChannels;

//16
typedef struct __attribute__((__packed__)){
	FrameDisplayChannels channels[8];
	FrameDisplayMode mode;
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
	uint32_t tx_bytes;				//	   0	- 65534
	uint32_t rx_bytes;				//	   0	- 65534
	uint16_t rx_error_frames;		//	   0	- 65534
	uint16_t recived_frames;		//	   0	- 65534
	uint16_t transmitted_frames;	//	   0	- 65534
	uint8_t	max_tran_deph;			//	   0	-	255
}FrameUartStat;

#endif
