#include <devices/lsm6dsl.h>
#include <devices/lps22hb.h>
#include <sdk/tick.h>
#include <sdk/debug.h>
#include "Led/led.h"
#include "Uart/uart.h"
#include <sdk/frame.h>
#include <math.h>
#include "Drv8301/drv8301.h"
#include <sdk/buzzer.h>
#include "Servo/servo.h"
#include <sdk/ahrs.h>
#include <sdk/rybos.h>
#include "Adc/adc.h"
#include <devices/si4468.h>
#include <stdio.h>
#include <string.h>
#include <sdk/log.h>
#include <sdk/radio.h>
#include <sdk/printf.h>
#include <sdk/uuid.h>
#include "Spi/spi.h"
#include "Bldc/bldc.h"
#include <settings/frame_frames.h>
#include <sdk/utils.h>
#include <sdk/atomic.h>
#include <sdk/pid.h>
#include <sdk/scope.h>

#define TASK_LED_PERIOD_MS							50
#define TASK_LED_BLINK_PERIOD						1000/TASK_LED_PERIOD_MS
#define TASK_SLEEP_PERIOD_MS						0
#define TASK_LOAD_MONITOR_PERIOD_S					5
#define TASK_LOAD_MONITOR_PERIOD_MS					1000*TASK_LOAD_MONITOR_PERIOD_S
#define TASK_BUZZER_PERIOD_MS						1
#define TASK_PRESSURE_READ_PERIOD_MS				1000/75
#define TASK_IMU_READ_PERIOD_MS						1000/((uint32_t)AHRS_SAMPLE_FREQUENCY_HZ)
#define TASK_BLDC_STATUS_PERIOD_MS					1
#define TASK_FRAME_DECODER_PERIOD_MS				10
#define TASK_RF_PERIOD_MS							0
#define TASK_PARAM_FAST_UPDATE_PERIOD_HZ			25
#define TASK_PARAM_SLOW_UPDATE_PERIOD_HZ			5
#define TASK_PARAM_FAST_UPDATE_PERIOD_MS			1000/TASK_PARAM_FAST_UPDATE_PERIOD_HZ
#define TASK_LOGGER_PERIOD_MS						1
#define TASK_RF_TIMEOUT_MS							0
#define TASK_PROTECTION_TIMEOUT_MS					10

#define UNDEVOLTAGE_SOUND_PERIOD_MS					1000
#define OVERTEMPERATURE_SOUND_PERIOD_MS				1000
#define START_BUTTON_LONG_PRESS_PERIOD_MS			2000

static float height_compensated = 0;
static float vel_compensated[3];
static float yaw_deg = 0;
static float pitch_deg = 0;
static float roll_deg = 0;
static bool rc_connected = false;
static float z_integrated_vel = 0;

static float yaw_start_ref_deg = 0;
static float height_start_ref = 0;

static int16_t rc_yaw = 0;
static int16_t rc_pitch = 0;
static int16_t rc_roll = 0;
static int16_t rc_throttle = 0;
static uint16_t rc_status = 0;

static float vel_offset[3] = { 0, 0, 0 };
static float vel_offset_tmp[3] = { 0, 0, 0 };
static uint32_t gyro_offset_cnt = 0;

#define PID_PITCH_ROLL_KP						0.0f
#define PID_PITCH_ROLL_KI						0.0f
#define PID_PITCH_ROLL_KD						0.0f
#define PID_PITCH_ROLL_OUT_LIMIT				0.0f
#define PID_PITCH_ROLL_D_FILTER					0.0f

#define PID_YAW_KP								0.5f
#define PID_YAW_KI								0.001f
#define PID_YAW_KD								100.0f
#define PID_YAW_OUT_LIMIT						25.0f
#define PID_YAW_D_FILTER						0.8f

#define PID_HEIGHT_KP							0.0f
#define PID_HEIGHT_KI							0.0f
#define PID_HEIGHT_KD							0.0f
#define PID_HEIGHT_OUT_LIMIT					0.0f
#define PID_HEIGHT_D_FILTER						0.0f

CCMRAM_VARIABLE PID_DEF(pid_pitch, PID_PITCH_ROLL_KP, PID_PITCH_ROLL_KI, PID_PITCH_ROLL_KD, PID_PITCH_ROLL_OUT_LIMIT, PID_PITCH_ROLL_D_FILTER);
CCMRAM_VARIABLE PID_DEF(pid_roll, PID_PITCH_ROLL_KP, PID_PITCH_ROLL_KI, PID_PITCH_ROLL_KD, PID_PITCH_ROLL_OUT_LIMIT, PID_PITCH_ROLL_D_FILTER);
CCMRAM_VARIABLE PID_DEF(pid_yaw, PID_YAW_KP, PID_YAW_KI,PID_YAW_KD, PID_YAW_OUT_LIMIT, PID_YAW_D_FILTER);
CCMRAM_VARIABLE PID_DEF(pid_height, PID_HEIGHT_KP, PID_HEIGHT_KI, PID_HEIGHT_KD, PID_HEIGHT_OUT_LIMIT, PID_HEIGHT_D_FILTER);

static void print_bldc_status(uint16_t status_reg1, uint16_t status_reg2) {
	//TODO add actions depends of fault
	//TODO add frame send to master

	if (status_reg1 & DRV8301_SR1_FAULT) {
		printf("DRV8310 Fault: FAULT\n");
	}

	if (status_reg1 & DRV8301_SR1_GVDD_UV) {
		printf("DRV8310 Fault: GVDD_UV (DRV8301 Vdd, Under Voltage)\n");
	}

	if (status_reg1 & DRV8301_SR1_PVDD_UV) {
		printf("DRV8310 Fault: VDD_UV (Power supply Vdd, Under Voltage)\n");
	}

	if (status_reg1 & DRV8301_SR1_OTSD) {
		printf("DRV8310 Fault: OTSD (Over Temperature Shut Down)\n");
	}

	if (status_reg1 & DRV8301_SR1_OTW) {
		printf("DRV8310 Fault: OTW (Over Temperature Warning)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETHA_OC) {
		printf("DRV8310 Fault: FETHA_OC (FET High side, Phase A Over Current)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETLA_OC) {
		printf("DRV8310 Fault: FETLA_OC (FET Low side, Phase A Over Current)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETHB_OC) {
		printf("DRV8310 Fault: FETHB_OC (FET High side, Phase B Over Current)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETLB_OC) {
		printf("DRV8310 Fault: FETLB_OC (FET Low side, Phase B Over Current)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETHC_OC) {
		printf("DRV8310 Fault: FETHC_OC (FET High side, Phase C Over Current)\n");
	}

	if (status_reg1 & DRV8301_SR1_FETLC_OC) {
		printf("DRV8310 Fault: FETLC_OC (FET Low side, Phase C Over Current)\n");
	}

	if (status_reg2 & DRV8301_SR2_GVDD_OV) {
		printf("DRV8310 Fault: GVDD_OV (DRV8301 Vdd, Over Voltage)\n");
	}
}

static void print_init(void) {
	printf("Slave Initialization finished...\n");
	printf("Compilation date: %s\n", __DATE__);
	printf("Compilation time: %s\n", __TIME__);
	printf("UUID0: %08X %08X %08X\n", (unsigned int) STM32_UUID[0], (unsigned int) STM32_UUID[1], (unsigned int) STM32_UUID[2]);

	FrameReqDisplayChannels frame;

	//Slave->Master->PC
	uart_send_frame(FRAME_TYPE_REQ_DISPLAY_CHANNELS, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);

	//TODO send KP KI KD to master and PC
}

static void print_cpu_load(void) {
	FrameSystemLoadSlave frame;

	uint32_t i;
	float accumulate = 0;
	float task_load;
	float cnt;

	printf("----------------------------------------\n");
	printf("--------------Load Monitor--------------\n");
	printf("I\\T\tID\tLoad[%%]\tExec. Cnt\tLabel\n");
	printf("----------------------------------------\n");

	for (i = 0; i < RYBOS_TASK_AND_IRQ_SIZE - 1; i++) {
		task_load = rybos_get_task_execution_time(i) / (TICK_CPU_FREQUENCY_HZ / 1000000);
		rybos_clear_task_execution_time(i);

		task_load /= 10000.0f * (float) TASK_LOAD_MONITOR_PERIOD_S;
		frame.load[i] = SCALE_FLOAT_TO_UINT16(task_load, 0.0f, 100.0f);

		accumulate += task_load;

		cnt = (float) rybos_get_task_execution_cnt(i) / (float) TASK_LOAD_MONITOR_PERIOD_S;
		rybos_clear_task_execution_cnt(i);

		uint8_t symbol;
		if (i < RYBOS_MARKER_IRQ_SIZE) {
			symbol = 'I';
		} else {
			symbol = 'T';
		}
		printf("%c\t%u\t%6.3f\t%8.1f\t%s\n", (char) symbol, (unsigned int) i + 1, (double) task_load, (double) cnt, RYBOS_IRQ_TASK_NAMES[i]);
	}

	task_load = 100.0f - accumulate;
	frame.load[i] = SCALE_FLOAT_TO_UINT16(task_load, 0.0f, 100.0f);

	cnt = (float) rybos_get_task_execution_cnt(i) / (float) TASK_LOAD_MONITOR_PERIOD_S;
	rybos_clear_task_execution_cnt(i);

	printf("T\t%u\t%6.3f\t%8.1f\t%s\n", (unsigned int) i + 1, (double) task_load, (double) cnt, RYBOS_IRQ_TASK_NAMES[i]);

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SYSTEM_LOAD_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_reset_status(void) {
	if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST)) {
		printf("Start: Low Power reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST)) {
		printf("Start: Window Watchdog reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) {
		printf("Start: Independent Watchdog reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_SFTRST)) {
		printf("Start: Software reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_PORRST)) {
		printf("Start: POR/PDR reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_PINRST)) {
		printf("Start: Pin reset\n");
	}

	if (RCC_GetFlagStatus(RCC_FLAG_OBLRST)) {
		printf("Start: Option Byte Loader reset \n");
	}

	//Clear all flags
	RCC_ClearFlag();
}

static void print_radio_parameters(void) {
	FrameRadioStat frame;

	frame.avarage_rssi = SCALE_FLOAT_TO_UINT16(radio_get_avarage_rssi(), -256.0f, 32.0f);						//Scale -256 - 32
	frame.max_rssi = SCALE_FLOAT_TO_UINT16(radio_get_max_rssi(), -256.0f, 32.0f);								//Scale -256 - 32
	frame.min_rssi = SCALE_FLOAT_TO_UINT16(radio_get_min_rssi(), -256.0f, 32.0f);								//Scale -256 - 32

	frame.max_tran_deph = radio_get_max_queue_depth();
	frame.rettransmition = radio_get_retransmition_cnt();
	frame.rx_bytes = radio_get_received_bytes();
	frame.tx_bytes = radio_get_transmited_bytes();
	frame.recived_frame = radio_get_received_frame();
	frame.recived_frame_total = radio_get_received_frame_total();
	frame.transmitted_frame = radio_get_transmitted_frame();

	radio_reset_statistics();

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_RADIO_STAT, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_system_param(void) {
	FrameSystemParamSlave frame;

	frame.critical_deph = critical_get_max_queue_depth();
	frame.spi_tran_deph = spi_get_max_queue_depth();
	frame.system_local_time = tick_get_time_ms();
	frame.scope_deph = scope_get_max_queue_depth();

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SYSTEM_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_uart_param(void) {
	FrameUartStat frame;

	frame.tx_bytes = uart_get_transmitted_bytes();
	frame.rx_bytes = uart_get_received_bytes();
	frame.rx_error_frames = uart_get_received_error_frames();
	frame.recived_frames = uart_get_received_frames();
	frame.transmitted_frames = uart_get_transmitted_frames();
	frame.max_tran_deph = uart_get_max_queue_depth();

	uart_reset_statistics();

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_UART_STAT, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_fast_param(void) {
	FrameFastParamsSlave frame;

	float *q = ahrs_get_q2();
	float *angle = servo_get_angle();

	//AHRS
	frame.ahrs.q[0] = SCALE_FLOAT_TO_INT16(q[0], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[1] = SCALE_FLOAT_TO_INT16(q[1], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[2] = SCALE_FLOAT_TO_INT16(q[2], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[3] = SCALE_FLOAT_TO_INT16(q[3], -1.0f, 1.0f);															//	  -1 	-     1

	//Servo
	frame.servo.angle[0] = SCALE_FLOAT_TO_INT16(angle[0], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[1] = SCALE_FLOAT_TO_INT16(angle[1], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[2] = SCALE_FLOAT_TO_INT16(angle[2], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[3] = SCALE_FLOAT_TO_INT16(angle[3], -180.0f, 180.0f);												//	-180 	-  	180		deg

	//Motor
	frame.motor.iq_current = SCALE_FLOAT_TO_INT16(bldc_get_i_q(), -50.0f, 50.0f);										//	-50 	-  	50		A
	frame.motor.rps = SCALE_FLOAT_TO_INT16(bldc_get_speed_rps(), -500.0f, 500.0f);										//	-10 	-  	50		A

	//PID
	frame.pid.pitch_e = SCALE_FLOAT_TO_INT16(0 - pitch_deg, -180.0f, 180.0f);											//	-180 	-  	180		deg
	frame.pid.roll_e = SCALE_FLOAT_TO_INT16(0 - roll_deg, -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.pid.yaw_e = SCALE_FLOAT_TO_INT16(0 - z_integrated_vel, -180.0f, 180.0f);										//	-180 	-  	180		deg
	frame.pid.height_e = SCALE_FLOAT_TO_INT16((height_start_ref - height_compensated), -50.0f, 50.0f);					//	-50 	-  	50		m

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_FAST_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_slow_param(void) {
	FrameSlowParamSlave frame;

	//ADC
	frame.adc.ntc_temp = SCALE_FLOAT_TO_UINT16(bldc_get_ntc_temperature_c(), -128.0f, 256.f);							//	-128 	- 	256		C
	frame.adc.up_temp = SCALE_FLOAT_TO_UINT16(bldc_get_up_temperature_c(), -128.0f, 256.0f);							//	-128 	- 	256		C
	frame.adc.bat_v = SCALE_FLOAT_TO_UINT16(bldc_get_v_vcc_v(), 0.0f, 32.0f);											//	   0 	-  	 32 	V
	frame.adc.ldo_v = SCALE_FLOAT_TO_UINT16(bldc_get_v_ldo_v(), 0.0f, 4.0f);											//	   0 	-     4		V

	//IMU
	float *acc = lsm6dsl_get_imu_acceleration();

	frame.imu.acc[0] = SCALE_FLOAT_TO_INT16(acc[0], -80.0f, 80.0f);														//	 -80 	-    80		m/s2
	frame.imu.acc[1] = SCALE_FLOAT_TO_INT16(acc[1], -80.0f, 80.0f);														//	 -80 	-    80		m/s2
	frame.imu.acc[2] = SCALE_FLOAT_TO_INT16(acc[2], -80.0f, 80.0f);														//	 -80 	-    80		m/s2

	frame.imu.vel[0] = SCALE_FLOAT_TO_INT16(vel_compensated[0], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s
	frame.imu.vel[1] = SCALE_FLOAT_TO_INT16(vel_compensated[1], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s
	frame.imu.vel[2] = SCALE_FLOAT_TO_INT16(vel_compensated[2], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s

	frame.imu.temp = SCALE_FLOAT_TO_UINT16(lsm6dsl_get_temperature_c(), -128.0f, 256.0f);								//	-128 	- 	256		C

	frame.pressure.height = SCALE_FLOAT_TO_INT16(height_compensated, -1000.0f, 1000.0f);								//	 -1000 	-  1000		m
	frame.pressure.press = SCALE_FLOAT_TO_UINT16(lps22hb_get_pressure_pa(), 90000.0f, 110000.0f);						//	 90k	-  110k 	Pa
	frame.pressure.temp = SCALE_FLOAT_TO_UINT16(lps22hb_get_temperature_c(), -128.0f, 256.0f);							//	-128 	- 	256		C

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SLOW_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}


static void print_error_history(void){
	FrameErrorLog frame;

	memcpy(frame.error, debug_get_last_error(), sizeof(FrameErrorLog));

	//Slave->Master->PC
	uart_send_frame(FRAME_TYPE_ERROR_LOG, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

void pid_to_frame_setting(Pid *pid, FrameSetPidSettings *frame) {
	frame->kp = pid->kp;
	frame->ki = pid->ki;
	frame->kd = pid->kd;
	frame->out_limit = pid->out_limit;
	frame->d_filter_coeff = pid->d_filter_coeff;
}

void reponse_pid_settings(FramePidType type) {
	//Response frame
	FrameSetPidSettings response = {
			.pid_type = type,
			.kp = 0,
			.ki = 0,
			.kd = 0,
			.out_limit = 0,
			.d_filter_coeff = 0,
	};

	switch (type) {
	case FRAME_PID_TYPE_PITCH_ROLL:
		//Response with only one, rest pitch is the same
		pid_to_frame_setting(&pid_roll, &response);
		break;

	case FRAME_PID_TYPE_YAW:
		pid_to_frame_setting(&pid_yaw, &response);
		break;

	case FRAME_PID_TYPE_HEIGHT:
		pid_to_frame_setting(&pid_height, &response);
		break;

	case FRAME_PID_TYPE_BLDC_SPEED:
		pid_to_frame_setting(bldc_get_pid(FRAME_PID_TYPE_BLDC_SPEED), &response);
		break;

	case FRAME_PID_TYPE_BLDC_DQ:
		pid_to_frame_setting(bldc_get_pid(FRAME_PID_TYPE_BLDC_DQ), &response);
		break;

	default:
		debug_error(PID_TYPE_NOT_SUPPORTED);
		break;
	}

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SET_PID_SETTINGS, (uint8_t *) (&response), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void rc_disconnected(void) {
	rc_yaw = 0;
	rc_pitch = 0;
	rc_roll = 0;
	rc_throttle = 0;

	//Stop motor
	bldc_stop_sig();

	//TODO implement auto landing
}

static void control_drone(void) {
	float yaw_control;
	float pitch_control;
	float roll_control;
	float dt = 1.0f / ((float) TASK_IMU_READ_PERIOD_MS);

	if (bldc_get_active_state() == BLDC_STATE_FOC) {
		//Control YAW
		z_integrated_vel += vel_compensated[2] * (float) TASK_IMU_READ_PERIOD_MS / 1000.0f;
		yaw_control = -pid_control_pid(&pid_yaw, z_integrated_vel, 0, dt);
		yaw_control -= pid_yaw.out_limit;
	} else {
		//Control YAW
		yaw_control = -pid_control_pid(&pid_yaw, 0, 0, dt);
	}

	//Control pitch
	pitch_control = -pid_control_pid(&pid_pitch, pitch_deg, (float)rc_roll * 25.0f / 2048.0f, dt);

	//Control roll
	roll_control = -pid_control_pid(&pid_roll, roll_deg, (float)-rc_pitch * 25.0f / 2048.0f, dt);

	//Combine all together
	float servo_top = 0;
	float servo_botom = 0;
	float servo_left = 0;
	float servo_right = 0;

	servo_top += yaw_control;
	servo_botom += yaw_control;
	servo_left += yaw_control;
	servo_right += yaw_control;

	servo_top -= roll_control;
	servo_botom += roll_control;

	servo_left -= pitch_control;
	servo_right += pitch_control;

	//float ch1 = z_integrated_vel * 10.0f;
	//float ch2 = pitch_control * 10.0f;
	//float ch3 = roll_control * 10.0f;
	//float ch4 = 0;

	//scope_send_4ch((int16_t) ch1, (int16_t) ch2, (int16_t) ch3, (int16_t) ch4);

	//Set servo values
	servo_set_position_angle(SERVO_POSITION_2_TOP, servo_top);
	servo_set_position_angle(SERVO_POSITION_4_BOTTOM, servo_botom);
	servo_set_position_angle(SERVO_POSITION_1_LEFT, servo_left);
	servo_set_position_angle(SERVO_POSITION_3_RIGHT, servo_right);
}

static void motor_start_stop_detection(uint16_t status) {
	//Check if unlocked
	if (!(status & FRAME_STATUS_LOCK)) {
		//Start motor
		if (status & FRAME_STATUS_BUTTON_RB) {
			bldc_start_sig();

			//Take reference YAW angle
			yaw_start_ref_deg = yaw_deg;
			z_integrated_vel = 0;

			//Take reference of height
			height_start_ref = height_compensated;
		}

		//Preventive stop
		if (status & FRAME_STATUS_BUTTON_LB) {
			z_integrated_vel = 0;
			pid_reset(&pid_yaw);
			bldc_stop_sig();
		}

		//Calibrate gyro
		if (status & FRAME_STATUS_BUTTON_RT) {
			vel_offset[0] = 0;
			vel_offset[1] = 0;
			vel_offset[2] = 0;
			vel_offset_tmp[0] = 0;
			vel_offset_tmp[1] = 0;
			vel_offset_tmp[2] = 0;
			gyro_offset_cnt = 0;
		}
	}
}

static void task_rf_timeout(void) {
	radio_timeout();
}

static void task_sleep(void) {
	//TODO implement all sleep functions

	//Prevent running scheduler after BLDC IRQ
	do {
		rybos_clear_irq_execution_mask();
		__WFI();
	} while (rybos_get_irq_execution_mask() == RYBOS_IRQ_UNIQUE_MASK_ADC_BLDC);

}

static void task_buzzer(void) {
	buzzer_state_machine();
}

static void task_logger(void) {
	log_tx_state_mashine();
}

static void task_rf(void) {
	radio_slave_sm();
}

static void task_frame_decoder(void) {
	//Send data
	scope_state_machine();

	//Read data
	uint8_t tmp;
	while (uart_get_byte_dma(&tmp)) {
		frame_decoding_state_mashine(tmp);
	}
}

static void task_bldc_status(void) {
	print_bldc_status(drv8301_get_status_reg1(), drv8301_get_status_reg2());

	drv8301_read_status();
}

static void task_led(void) {
	//Led status diode
	static uint32_t counter = 0;

	if (counter == 1) {
		//TODO remove comment
		//LED_BLUE_ON;
	} else if (counter == TASK_LED_BLINK_PERIOD) {
		counter = 0;
	} else {
		//TODO remove comment
		//LED_BLUE_OFF;
	}
	counter++;

	//Led Red disabler
	static bool led_red_on = false;

	if (led_red_on) {
		LED_RED_OFF;
		led_red_on = false;
	}
	led_red_on = LED_RED_CHECK;

	//Connection status
	if (counter == 1) {
		static uint32_t frame_cnt = 0;
		if (radio_get_received_frame_total() - frame_cnt >= RADIO_FRAME_PER_S_CONNECTION_LIMIT) {
			rc_connected = true;
		} else {
			rc_connected = false;
			rc_disconnected();
		}
		frame_cnt = radio_get_received_frame_total();
	}
}

static void task_read_pressure(void) {
	static float height_offset = 0;
	static float height_tmp = 0;
	static uint32_t height_offset_cnt = 0;

	lps22hb_read_sensor();

	//Offset compensation
	if (height_offset_cnt < LPS22HB_OFFSET_COUNTER) {
		height_tmp += lps22hb_get_height_m();
		height_offset_cnt++;
	} else if (height_offset_cnt == LPS22HB_OFFSET_COUNTER) {
		height_offset = height_tmp / (float) height_offset_cnt;
		height_offset_cnt++;
	}

	height_compensated = lps22hb_get_height_m() - height_offset;
}

static void task_imu_read(void) {
	float *acc = lsm6dsl_get_imu_acceleration();
	float *vel = lsm6dsl_get_angular_velocity();

	//Offset compensation
	if (gyro_offset_cnt < IMU_OFFSET_COUNTER) {
		vel_offset_tmp[0] += vel[0];
		vel_offset_tmp[1] += vel[1];
		vel_offset_tmp[2] += vel[2];
		gyro_offset_cnt++;
	} else if (gyro_offset_cnt == IMU_OFFSET_COUNTER) {
		vel_offset[0] = vel_offset_tmp[0] / (float) gyro_offset_cnt;
		vel_offset[1] = vel_offset_tmp[1] / (float) gyro_offset_cnt;
		vel_offset[2] = vel_offset_tmp[2] / (float) gyro_offset_cnt;
		gyro_offset_cnt++;
		ahrs_set_beta(AHRS_BETA_FINAL);

		//Finish calibration sound
		buzzer_generate_sound(BUZZER_SOUND_DOUBLE_RISING);
	}

	vel_compensated[0] = vel[0] - vel_offset[0];
	vel_compensated[1] = vel[1] - vel_offset[1];
	vel_compensated[2] = vel[2] - vel_offset[2];

	//AHRS update
	ahrs_update(DEG_TO_RAD(vel_compensated[0]), DEG_TO_RAD(-vel_compensated[1]), DEG_TO_RAD(-vel_compensated[2]), acc[0], -acc[1], -acc[2]); //YPR 0 0 0

	ahrs_rotate_45();

	//toll <--> yaw
	roll_deg = ahrs_get_yaw();
	pitch_deg = ahrs_get_pitch();
	yaw_deg = ahrs_get_roll();

	lsm6dsl_read_sensor();

	control_drone();
}

static void task_param_update(void) {
	static uint32_t cnt = 0;

	//25Hz
	print_fast_param();

	//5Hz
	if (cnt == 0) {
		print_slow_param();
		print_error_history();
	}

	cnt++;
	if (cnt == TASK_PARAM_FAST_UPDATE_PERIOD_HZ / TASK_PARAM_SLOW_UPDATE_PERIOD_HZ) {
		cnt = 0;
	}
}

static void task_protection(void) {
	//Under-voltage protection
	if (bldc_get_v_vcc_v() < ADC_MIN_BAT_V) {
		static uint32_t time = 0;

		if (tick_get_time_ms() - time > UNDEVOLTAGE_SOUND_PERIOD_MS) {
			time = tick_get_time_ms();
			buzzer_generate_sound(BUZZER_SOUND_SINGLE_PEAK);
		}
	}

	//Over-temperature protection
	if (bldc_get_ntc_temperature_c() > ADC_NTC_MAX_TEMP_C) {
		static uint32_t time = 0;

		if (tick_get_time_ms() - time > OVERTEMPERATURE_SOUND_PERIOD_MS) {
			time = tick_get_time_ms();
			buzzer_generate_sound(BUZZER_SOUND_DOUBLE_PEAK);
		}
	}
}

static void task_load_monitor(void) {
	print_cpu_load();
	print_radio_parameters();
	print_system_param();
	print_uart_param();
}

void frame_cb_req_init_data(void *buff, uint8_t params) {
	FrameRespInitData frame;

	memcpy(frame.compilation_date, __DATE__, 11);
	memcpy(frame.compilation_time, __TIME__, 8);

	frame.magic_init_number = INIT_MAGIC_NUMBER;
	frame.uuid[0] = STM32_UUID[0];
	frame.uuid[1] = STM32_UUID[1];
	frame.uuid[2] = STM32_UUID[2];

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_RESP_INIT_DATA, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);

	UNUSED(buff);
	UNUSED(params);
}

void frame_cb_rc_control(void *buff, uint8_t params) {
	FrameRcControl *frame = (FrameRcControl *) buff;

	if (rc_connected) {
		rc_yaw = frame->yaw;
		rc_pitch = frame->pitch;
		rc_roll = frame->roll;
		rc_throttle = frame->throttle;
		rc_status = frame->status;

		motor_start_stop_detection(rc_status);
		bldc_increase_motor_speed_rps(rc_throttle / 2048.0f * 1.0f);
	} else {
		rc_disconnected();
	}

	UNUSED(params);
}

void frame_cb_get_pid_settings(void *buff, uint8_t params) {
	FrameGetPidSettings *frame = (FrameGetPidSettings *) buff;

	reponse_pid_settings(frame->pid_type);

	UNUSED(params);
}

void frame_cb_set_pid_settings(void *buff, uint8_t params) {
	FrameSetPidSettings *frame = (FrameSetPidSettings *) buff;

	switch (frame->pid_type) {
	case FRAME_PID_TYPE_PITCH_ROLL:
		pid_set_param(&pid_pitch, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		pid_set_param(&pid_roll, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		break;

	case FRAME_PID_TYPE_YAW:
		pid_set_param(&pid_yaw, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		break;

	case FRAME_PID_TYPE_HEIGHT:
		pid_set_param(&pid_height, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		break;

	case FRAME_PID_TYPE_BLDC_SPEED:
		bldc_set_pid(FRAME_PID_TYPE_BLDC_SPEED, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		break;

	case FRAME_PID_TYPE_BLDC_DQ:
		bldc_set_pid(FRAME_PID_TYPE_BLDC_DQ, frame->kp, frame->ki, frame->kd, frame->out_limit, frame->d_filter_coeff);
		break;

	default:
		debug_error(PID_TYPE_NOT_SUPPORTED);
		break;
	}

	//Send back settings
	reponse_pid_settings(frame->pid_type);

	UNUSED(params);
}

void frame_received_complete(FrameType type, FrameParams params, uint8_t *buff, void (*cb_handler)(void *, uint8_t)) {
	uart_increment_reveived_frame_cnt();
	if (params & FRAME_DESTINATION_MASTER_PC) {
		//Forward received frame to slave not allowed
		//radio_send_frame(type, (uint8_t *) (buff), params);
	} else {
		//Call original frame VB
		cb_handler((void *) (buff), params);
	}

	UNUSED(type);
}

//Frame received
void frame_received_error(void) {
	uart_increment_received_error_frame_cnt();
	debug_error(FRAME_CRC_ERROR);
}

/*
 * ####################################################################################################################
 * Name		| IRQ		| P	| E | Peripherals 	| Finished 	| To do
 * ####################################################################################################################
 * ADC		|			| 3	|	|				|	-		| ????
 * --------------------------------------------------------------------------------------------------------------------
 * Ahrs		| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * BLDC		|			|	|	|				|	-		| ????
 * --------------------------------------------------------------------------------------------------------------------
 * Buzzer	| -			| -	| -	| TIM2			|	+		| Hardware fix in the future (PA0<->PA1) - tick need to wake up the uP - TIM2(32b) <-> TIM15(16b)
 * --------------------------------------------------------------------------------------------------------------------
 * Cyclic	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Debug	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Drv8301	| TIM1		| 0	| -	| TIM1			|	+		| drv8301_fault_irq implementation
 * --------------------------------------------------------------------------------------------------------------------
 * Frame	|			|	|	|				|	+-		| Ongoing, add MS and FS
 * --------------------------------------------------------------------------------------------------------------------
 * Imu		| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Led		| -			| -	| -	| -				|	+		| Led blue uncomment, add PWM to LEDS
 * --------------------------------------------------------------------------------------------------------------------
 * Log		| -			| -	| -	| ITM			|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * PID		| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Pressure | -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Printf	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Radio	| -			| -	| -	| -				|	-		| TODO Refactor
 * --------------------------------------------------------------------------------------------------------------------
 * Rybos	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Scope	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Servo	| -			| -	| -	| TIM4			|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * SI4468	| EXTI		| 7	| -	| PB5			|	+		| -
 *  		| 			| 7	| -	| PB10			|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Spi		| DMA1_CH2 	| 5	| -	| DMA1_CH2		|	+		| Unlock GPIO comment RF NSS
 * 			| 			| -	| -	| DMA1_CH3		|			| Check if asm not is really needed
 * --------------------------------------------------------------------------------------------------------------------
 * Tick		| SysTick	| 4	| -	| SystTick		|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Uart		| DMA1_CH7	| 8	| -	| USART2		|	-		| -
 * --------------------------------------------------------------------------------------------------------------------
 * ####################################################################################################################
 * TODO add transaction complete flag to transaction record
 * TODO PKT_LEN - rx packet len put into the fifo IN_FIFO
 * TODO AN626 PKT_LEN_ADJUST - way to reach 64 byte fifo rx
 * TODO PKT_FIELD_2_LENGTH_7_0 - max expected rx frame - now 64
 * TODO when Master send last ack it now wait for the last frame from slave
 * TODO prriority for adc shuld be low - glitches visible at L measurement
 * TODO servo preload register enable
 * TODO add crc to radio frames- risk of receiving large fifo with random data
 * TODO cleare only flag which was read in the Radio
 * TODO si4468 adc and temperature sensor
 * TODO RSSI hopping
 * TODO RSSI latch
 * TODO Max packet length
 * TODO whitening
 * TODO clear rx fifo
 * INFO PKT_FIELD_2_CRC_CONFIG wrong generated, should be in rx i tx 0x2A
 * TODO remove float from all irq
 * TODO try to use embedded timer to time base
 * TODO watchodg implelemntation
 * TODO window watchdog na irq BLDC
 * TODO replace all modulo by if else
 * TODO adc VBAT remove
 * TODO add received frame, add received frame total
 * TODO update frame boudrate estimation
 * TODO remove in IMU division /
 * TODO update table
 * TODO add to frame PID ID defines
 * TODO unique name for all transactions
 * TODO implement reset of sensors like a pressure ang imu after startup
 * TODO chenge the priority of systick to have the highest one
 * TODO chenge critical functions to inline
 * TODO add two separate section of CMM, 1 for variables, 1 for functions. Go to AN4296
 * TOOD add dead time compensation to RL measurement
 * TODO 10x mniejsze wmocnienie dla nizszysch predkosci
 * TODO add preload for ARR and CCR for all signals
 * TODO use ADC interrupt to measure time
 * TODO BLDC refactor to static inline functions
 * TODO BLDC refactor measurements functions
 * TODO combine si4468 with radio
 * TODO in all transaction replace size by sizeof
 */

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	log_init();
	print_reset_status();
	tick_init();
	led_init();
	buzzer_init();
	uart_init();
	lsm6dsl_init();
	lps22hb_init();
	servo_init();
	bldc_init();
	adc_init();
	drv8301_init();
	radio_init();

	rybos_add_task(TASK_IMU_READ_PERIOD_MS, 		   8, task_imu_read, 		RYBOS_MARKER_TASK_IMU_READ, 		true);
	rybos_add_task(TASK_BLDC_STATUS_PERIOD_MS, 		  63, task_bldc_status, 	RYBOS_MARKER_TASK_BLDC_STATUS, 		true);
	rybos_add_task(TASK_PROTECTION_TIMEOUT_MS, 		  65, task_protection, 		RYBOS_MARKER_TASK_PROTECTION, 		true);
	rybos_add_task(TASK_PRESSURE_READ_PERIOD_MS, 	  65, task_read_pressure, 	RYBOS_MARKER_TASK_PRESSURE_READ, 	true);
	rybos_add_task(TASK_RF_TIMEOUT_MS, 				 124, task_rf_timeout, 		RYBOS_MARKER_TASK_RF_TIMEOUT, 		false);
	rybos_add_task(TASK_FRAME_DECODER_PERIOD_MS, 	 126, task_frame_decoder, 	RYBOS_MARKER_TASK_FRAME_DECODER, 	true);
	rybos_add_task(TASK_BUZZER_PERIOD_MS, 			 127, task_buzzer, 			RYBOS_MARKER_TASK_BUZZER, 			false);
	rybos_add_task(TASK_LED_PERIOD_MS, 				 245, task_led, 			RYBOS_MARKER_TASK_LED, 				true);
	rybos_add_task(TASK_LOAD_MONITOR_PERIOD_MS, 	 230, task_load_monitor, 	RYBOS_MARKER_TASK_LOAD_MONITOR, 	true);
	rybos_add_task(TASK_SLEEP_PERIOD_MS, 			 250, task_sleep, 			RYBOS_MARKER_TASK_SLEEP, 			true);
	rybos_add_task(TASK_RF_PERIOD_MS, 				 125, task_rf, 				RYBOS_MARKER_TASK_RF, 				true);
	rybos_add_task(TASK_PARAM_FAST_UPDATE_PERIOD_MS, 220, task_param_update, 	RYBOS_MARKER_TASK_PARAM_UPDATE, 	true);
	rybos_add_task(TASK_LOGGER_PERIOD_MS, 			 240, task_logger, 			RYBOS_MARKER_TASK_LOGGER, 			true);

	print_init();
	buzzer_generate_sound(BUZZER_SOUND_START);
	while (1) {
		rybos_scheduler_run();
	}
}
