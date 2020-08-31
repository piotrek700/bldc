#include "main.h"
#include "Tick/tick.h"
#include "Debug/debug.h"
#include "Led/led.h"
#include "Uart/uart.h"
#include "Frame/frame.h"
#include "math.h"
#include "Drv8301/drv8301.h"
#include "Buzzer/buzzer.h"
#include "Pressure/pressure.h"
#include "Servo/servo.h"
#include "Imu/imu.h"
#include "Ahrs/ahrs.h"
#include "Rybos/rybos.h"
#include "Adc/adc.h"
#include "Si4468/si4468.h"
#include <stdio.h>
#include <string.h>
#include "Log/log.h"
#include "Radio/radio.h"
#include "Printf/printf.h"
#include "uuid.h"
#include "Spi/spi.h"
#include "Bldc/bldc.h"
#include "Frame/frame_frames.h"
#include "utils.h"
#include "Atomic/atomic.h"

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
#define TASK_LOGGER_PERIOD_MS						0
#define TASK_RF_TIMEOUT_MS							0

#define UNDEVOLTAGE_SOUND_PERIOD_MS					1000
#define OVERTEMPERATURE_SOUND_PERIOD_MS				1000
#define START_BUTTON_LONG_PRESS_PERIOD_MS			2000

static float height_compensated = 0;
static float vel_compensated[3];
static float yaw = 0;
static float pitch = 0;
static float roll = 0;
static bool rc_connected = false;

static int16_t rc_yaw = 0;
static int16_t rc_pitch = 0;
static int16_t rc_roll = 0;
static int16_t rc_throttle = 0;
static uint16_t rc_status = 0;
static uint32_t ii_uart_send = 0;

static void uart_send_scope_data(void) {
	//TODO refactor this
	while (bldc_get_frame_ready(ii_uart_send)) {
		uart_send_scope_frame(FRAME_TYPE_DISPLAY_CHANNELS_DATA_4, sizeof(FrameDisplayChannelsData4), (uint8_t *) bldc_get_scope_4ch_frame(ii_uart_send));
		bldc_get_frame_ready_clear(ii_uart_send);

		ii_uart_send++;
		if (ii_uart_send == BLDC_FRAME_SCOPE_BUFF_SIZE) {
			ii_uart_send = 0;
		}
	}
}

static void task_rf_timeout(void) {
	radio_timeout();
}

static void task_sleep(void) {
	//TODO implement all sleep functions

	//Prevent running scheduler after BLDC IRQ
	do{
		rybos_clear_irq_execution_mask();
		__WFI();
	}while(rybos_get_irq_execution_mask() == RYBOS_IRQ_UNIQUE_MASK_ADC_BLDC);

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
	uart_send_scope_data();

	//Read data
	uint8_t tmp;
	while (uart_get_byte_dma(&tmp)) {
		frame_decoding_state_mashine(tmp);
	}
}

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

static void task_bldc_status(void) {
	print_bldc_status(drv8301_get_status_reg1(), drv8301_get_status_reg2());

	drv8301_read_status();
}

static void rc_disconnected(void) {
	rc_yaw = 0;
	rc_pitch = 0;
	rc_roll = 0;
	rc_throttle = 0;

	//TODO implement auto landing
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
	static uint32_t frame_cnt = 0;
	if (counter == 1) {
		if (radio_get_received_frame_total() - frame_cnt >= RADIO_FRAME_PER_S_CONNECTION_LIMIT) {
			rc_connected = true;
			//TODO send KP KI KD to master and pc
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
	static uint32_t offset_cnt = 0;

	pressure_read_sensor();

	//Offset compensation
	if (offset_cnt < PRESSURE_OFFSET_COUNTER) {
		height_tmp += pressure_get_height_m();
		offset_cnt++;
	} else if (offset_cnt == PRESSURE_OFFSET_COUNTER) {
		height_offset = height_tmp / (float) offset_cnt;
		offset_cnt++;
	}

	height_compensated = pressure_get_height_m() - height_offset;
}

static void control_drone(void) {
	//TODO implement control_drone flight function
	//TODO implement control_drone for balance, rotation and height

	/*
	 servo_set_position_angle(SERVO_POSITION_2_TOP, drone_parameters.pitch);
	 servo_set_position_angle(SERVO_POSITION_4_BOTTOM, -drone_parameters.pitch);
	 servo_set_position_angle(SERVO_POSITION_1_LEFT, drone_parameters.yaw);
	 servo_set_position_angle(SERVO_POSITION_3_RIGHT, -drone_parameters.yaw);
	 */

	/* Rotation
	 servo_set_position_angle(SERVO_POSITION_2_TOP, -drone_parameters.roll);
	 servo_set_position_angle(SERVO_POSITION_4_BOTTOM,+drone_parameters.roll);
	 servo_set_position_angle(SERVO_POSITION_1_LEFT, +drone_parameters.roll);
	 servo_set_position_angle(SERVO_POSITION_3_RIGHT,-drone_parameters.roll);
	 */

	float tmp = 0;
	tmp = (float)rc_pitch * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_2_TOP, tmp);

	tmp = (float)rc_pitch * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_4_BOTTOM, tmp);

	tmp = (float)rc_pitch * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_1_LEFT, tmp);

	tmp = (float)rc_pitch * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_3_RIGHT, tmp);
}

static void task_imu_read(void) {
	static float vel_offset[3] = { 0, 0, 0 };
	static float vel_offset_tmp[3] = { 0, 0, 0 };
	static uint32_t offset_cnt = 0;

	float *acc = imu_get_imu_acceleration();
	float *vel = imu_get_angular_velocity();

	//Offset compensation
	if (offset_cnt < IMU_OFFSET_COUNTER) {
		vel_offset_tmp[0] += vel[0];
		vel_offset_tmp[1] += vel[1];
		vel_offset_tmp[2] += vel[2];
		offset_cnt++;
	} else if (offset_cnt == IMU_OFFSET_COUNTER) {
		vel_offset[0] = vel_offset_tmp[0] / (float) offset_cnt;
		vel_offset[1] = vel_offset_tmp[1] / (float) offset_cnt;
		vel_offset[2] = vel_offset_tmp[2] / (float) offset_cnt;
		offset_cnt++;
		ahrs_set_beta(AHRS_BETA_FINAL);
	}

	vel_compensated[0] = vel[0] - vel_offset[0];
	vel_compensated[1] = vel[1] - vel_offset[1];
	vel_compensated[2] = vel[2] - vel_offset[2];

	//AHRS update
	ahrs_update(DEG_TO_RAD(vel_compensated[0]), DEG_TO_RAD(-vel_compensated[1]), DEG_TO_RAD(-vel_compensated[2]), acc[0], -acc[1], -acc[2]); //YPR 0 0 0

	ahrs_rotate_45();

	yaw = ahrs_get_yaw();
	pitch = ahrs_get_pitch();
	roll = ahrs_get_roll();

	imu_read_sensor();

	control_drone();

	//Send to scope
	float ch1 = vel_compensated[0] * 1000.0f;
	float ch2 = vel_compensated[1] * 1000.0f;
	float ch3 = vel_compensated[2] * 1000.0f;
	float ch4 = vel[0] * 1000.0f;

	bldc_scope_send_data((int16_t) ch1, (int16_t) ch2, (int16_t) ch3, (int16_t) ch4);
}

void frame_cb_req_init_data(void *buff, uint8_t params) {
	UNUSED(buff);
	UNUSED(params);

	FrameRespInitData frame;

	memcpy(frame.compilation_date, __DATE__, 11);
	memcpy(frame.compilation_time, __TIME__, 8);

	frame.magic_init_number = INIT_MAGIC_NUMBER;
	frame.uuid[0] = STM32_UUID[0];
	frame.uuid[1] = STM32_UUID[1];
	frame.uuid[2] = STM32_UUID[2];

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_RESP_INIT_DATA, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_fast_param(void) {
	FrameFastParamsSlave frame;

	float *q = ahrs_get_q2();
	float *angle = servo_get_angle();

	frame.ahrs.q[0] = SCALE_FLOAT_TO_INT16(q[0], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[1] = SCALE_FLOAT_TO_INT16(q[1], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[2] = SCALE_FLOAT_TO_INT16(q[2], -1.0f, 1.0f);															//	  -1 	-     1
	frame.ahrs.q[3] = SCALE_FLOAT_TO_INT16(q[3], -1.0f, 1.0f);															//	  -1 	-     1

	frame.servo.angle[0] = SCALE_FLOAT_TO_INT16(angle[0], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[1] = SCALE_FLOAT_TO_INT16(angle[1], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[2] = SCALE_FLOAT_TO_INT16(angle[2], -180.0f, 180.0f);												//	-180 	-  	180		deg
	frame.servo.angle[3] = SCALE_FLOAT_TO_INT16(angle[3], -180.0f, 180.0f);												//	-180 	-  	180		deg

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
	float *acc = imu_get_imu_acceleration();

	frame.imu.acc[0] = SCALE_FLOAT_TO_INT16(acc[0], -80.0f, 80.0f);														//	 -80 	-    80		m/s2
	frame.imu.acc[1] = SCALE_FLOAT_TO_INT16(acc[1], -80.0f, 80.0f);														//	 -80 	-    80		m/s2
	frame.imu.acc[2] = SCALE_FLOAT_TO_INT16(acc[2], -80.0f, 80.0f);														//	 -80 	-    80		m/s2

	frame.imu.vel[0] = SCALE_FLOAT_TO_INT16(vel_compensated[0], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s
	frame.imu.vel[1] = SCALE_FLOAT_TO_INT16(vel_compensated[1], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s
	frame.imu.vel[2] = SCALE_FLOAT_TO_INT16(vel_compensated[2], -2000.0f, 2000.0f);										//   -2k 	-  	 2k		deg/s

	frame.imu.temp = SCALE_FLOAT_TO_UINT16(imu_get_temperature_c(), -128.0f, 256.0f);									//	-128 	- 	256		C

	frame.pressure.height = SCALE_FLOAT_TO_INT16(height_compensated, -1000.0f, 1000.0f);								//	 -1000 	-  1000		m
	frame.pressure.press = SCALE_FLOAT_TO_UINT16(pressure_get_pressure_pa(), 90000.0f, 110000.0f);						//	 90k	-  110k 	Pa
	frame.pressure.temp = SCALE_FLOAT_TO_UINT16(pressure_get_temperature_c(), -128.0f, 256.0f);							//	-128 	- 	256		C

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SLOW_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void undervoltage_protection(void) {
	static uint32_t time = 0;

	if (bldc_get_v_vcc_v() < ADC_MIN_BAT_V) {
		if (tick_get_time_ms() - time > UNDEVOLTAGE_SOUND_PERIOD_MS) {
			time = tick_get_time_ms();
			buzzer_generate_sound(BUZZER_SOUND_SINGLE_PEAK);
		}
	}
}

static void overtemperature_protection(void) {
	static uint32_t time = 0;

	if (bldc_get_ntc_temperature_c() > ADC_NTC_MAX_TEMP_C) {
		if (tick_get_time_ms() - time > OVERTEMPERATURE_SOUND_PERIOD_MS) {
			time = tick_get_time_ms();
			buzzer_generate_sound(BUZZER_SOUND_DOUBLE_PEAK);
		}
	}
}

static void task_param_update(void) {
	static uint32_t cnt = 0;

	//25Hz
	print_fast_param();

	//5Hz
	if (cnt == 0) {
		print_slow_param();
	}

	cnt++;
	if (cnt == TASK_PARAM_FAST_UPDATE_PERIOD_HZ / TASK_PARAM_SLOW_UPDATE_PERIOD_HZ) {
		cnt = 0;
	}

	//Protection
	//TODO separate thread
	undervoltage_protection();
	overtemperature_protection();
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
		if(i < RYBOS_MARKER_IRQ_SIZE){
			symbol = 'I';
		}else{
			symbol = 'T';
		}
		printf("%c\t%u\t%6.3f\t%8.1f\t%s\n", (char)symbol, (unsigned int) i + 1, (double) task_load,  (double) cnt, RYBOS_IRQ_TASK_NAMES[i]);
	}

	task_load = 100.0f - accumulate;
	frame.load[i] = SCALE_FLOAT_TO_UINT16(task_load, 0.0f, 100.0f);

	cnt = (float) rybos_get_task_execution_cnt(i) / (float) TASK_LOAD_MONITOR_PERIOD_S;
	rybos_clear_task_execution_cnt(i);

	printf("T\t%u\t%6.3f\t%8.1f\t%s\n", (unsigned int) i + 1, (double) task_load, (double) cnt, RYBOS_IRQ_TASK_NAMES[i]);

	//Slave->Master->PC
	radio_send_frame(FRAME_TYPE_SYSTEM_LOAD_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
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

	frame.critical_deh = critiacl_get_max_queue_depth();
	frame.spi_tran_deph = spi_get_max_queue_depth();
	frame.system_local_time = tick_get_time_ms();

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

static void task_load_monitor(void) {
	print_cpu_load();
	print_radio_parameters();
	print_system_param();
	print_uart_param();
}

void frame_received_complete(FrameType type, FrameParams params, uint8_t *buff, void (*cb_handler)(void *, uint8_t)){
	uart_increment_reveived_frame_cnt();
	if (params & FRAME_DESTINATION_MASTER_PC) {
		//Call original frame VB
		//PC->Slave
		cb_handler((void *) (buff), params);
	} else {
		//Forward received frame to slave
		//PC->Master->Slave
		radio_send_frame(type, (uint8_t *) (buff), params);
	}
}

//Frame received
void frame_received_error(void) {
	uart_increment_received_error_frame_cnt();
	debug_error(FRAME_CRC_ERROR);
}

static void motor_start_stop_detection(uint16_t status) {
	//Check if unlocked
	if (!(status & FRAME_STATUS_LOCK)) {
		//RB long press detection
		static uint32_t rb_press_timmer = 0;

		if (status & FRAME_STATUS_BUTTON_RB) {

			if (rb_press_timmer == 0) {
				rb_press_timmer = tick_get_time_ms();
			} else {
				if (tick_get_time_ms() - rb_press_timmer > START_BUTTON_LONG_PRESS_PERIOD_MS) {
					rb_press_timmer = 0;
					bldc_start_sig();
				}
			}
		} else {
			rb_press_timmer = 0;
		}

		//Preventive stop
		if (status & FRAME_STATUS_BUTTON_LB) {
			bldc_set_active_state(BLDC_STATE_STOP);
		}
	}
}

void frame_cb_rc_control(void *buff, uint8_t params) {
	UNUSED(params);

	FrameRcControl *frame = (FrameRcControl *) buff;

	if (rc_connected) {
		rc_yaw = frame->yaw;
		rc_pitch = frame->pitch;
		rc_roll = frame->roll;
		rc_throttle = frame->throttle;
		rc_status = frame->status;

		motor_start_stop_detection(rc_status);
		bldc_increase_motor_speed_rps(rc_throttle / 2048.0f * 1.0f);
		//angle_offset = (float) rc_roll / 2048.0f * (float)M_PI;
		//bldc_set_i_q_ref((float) rc_throttle / 2048.0f * 10.0f);

	} else {
		rc_disconnected();
	}
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
 * Pressure | -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Printf	| -			| -	| -	| -				|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Radio	| -			| -	| -	| -				|	-		| TODO
 * --------------------------------------------------------------------------------------------------------------------
 * Rybos	| -			| -	| -	| -				|	+		| Minimise critical section
 * --------------------------------------------------------------------------------------------------------------------
 * Servo	| -			| -	| -	| TIM4			|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * SI4468	| EXTI		| 7	| -	| PB5			|	+		| -
 *  		| 			| 7	| -	| PB10			|	+		| -
 * --------------------------------------------------------------------------------------------------------------------
 * Spi		| DMA1_CH2 	| 5	| -	| DMA1_CH2		|	+		| Unlock GPIO comment RF NSS
 * 			| 			| -	| -	| DMA1_CH3		|			| Check if asm not is really needed
 * --------------------------------------------------------------------------------------------------------------------
 * Tick		| SysTick	| 4	| -	| SystTick		|	+		| Replace by 32 bit timer - tim2 - not possible in this hardware(PA0<->PA1)
 * --------------------------------------------------------------------------------------------------------------------
 * Uart		| DMA1_CH7	| 8	| -	| USART2		|	-		| Optimization of frame generation
 * --------------------------------------------------------------------------------------------------------------------
 * ####################################################################################################################
 * TODO add transaction complete flag to transaction record
 * TODO after test remove all protection code
 * TODO PKT_LEN - rx packet len put into the fifo IN_FIFO
 * TODO AN626 PKT_LEN_ADJUST - way to reach 64 byte fifo rx
 * TODO PKT_FIELD_2_LENGTH_7_0 - max expected rx frame - now 64
 * TODO when Master send last ack it now wait for the last frame from slave
 * TODO CCMRAM for speed up the FOC implementation
 * TODO implement fast ahrs init based at acc measurements
 * TODO remove bldc status task and replace by IRQ
 * TODO integrate pressure scan with IMU
 * TODO check task priotities
 * TODO overrun enable in adc - remove or optimize flag clearing
 * TODO PWM_PERIOD_MAX = PWM_PERIOD +DEAT_TIME_CYCLES - max i kompensacja Dead time
 * TODO consider diffirent way of measurements bldc rs 2/3R
 * TODO prriority for adc shuld be low - glitches visible at L measurement
 * TODO servo preload register enable
 * TODO add crc to radio frames- risk of receiving large fifo with random data
 * TODO wolno stabilizuje sie Quaternion
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
 * TODO add dynamic PID with I active limitation
 * TODO 10x mniejsze wmocnienie dla nizszysch predkosci
 * TODO add preload for ARR and CCR for all signals
 * TODO use ADC interrupt to measure time
 * TODO after disconnect disable all control
 * TODO update RAD_TO_DEG and DEG_TO_RAD
 * TODO update to binary form 0b10
 * TODO BOD add
 * TODO add a init info about what generate a reset
 * TODO add wtd as a global
 * TODO add wtd as window comparator
 * TODO printf in foc measuremet limit number of decimal points
 * TODO rename the sensors files
 * TODO remove no callback buffers
 */


/*
STATIC_ASSERT(sizeof(enum lsm6ds3tr_reg) == sizeof(uint8_t), Unsupported_enum_lsm6ds3tr_reg);

#ifdef CONFIG_BIG_ENDIAN
#error "This module is not ready to work with BIG Endiannes"
#endif
*/
//TODO check if enum is uint8_t


/*
 *  1 | ADC			| No
 *  2 | AHRS		| Yes
 *  3 | Atomic		| Yes
 *  4 | BLDC		| No
 *  5 | Buzzer		| Yes
 *  6 | Cyclic		| Yes
 *  7 | Debug		| Yes
 *  8 | Drv8301		| No
 *  9 | Frame		| Yes
 * 10 | Imu			|
 * 11 | Led			| No
 * 12 | Log			| Yes
 * 13 | Pressure	|
 * 14 | Printf		| Yes
 * 15 | Radio		|
 * 16 | Rybos		| Yes
 * 17 | Servo		| No
 * 18 | Si4468		| No
 * 19 | Spi			| No
 * 20 | Tick		| Yes
 * 21 | Uart		| No
 * 22 | Utils		| Yes
 */



union lsm6ds3tr_reg {
	struct  {
	    uint8_t raw;
	}whoami_reg;

	struct {
		uint8_t ODR_XL :4;
		uint8_t FS_XL :2;
		uint8_t BW_XL :2;
	} ctrl1_xl;

	struct {
		uint8_t ODR_G :4;
		uint8_t FS_G :2;
		uint8_t FS_125 :1;
		uint8_t :1;
	} ctrl2_g;
	uint8_t raw;
};

union lsm6ds3tr_reg xxx[2] = {
		{.ctrl1_xl.ODR_XL = 5},
		{.ctrl1_xl.ODR_XL = 5},
};

//xxx[0].whoami_reg.raw = 7;

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	log_init();
	tick_init();
	led_init();
	buzzer_init();
	uart_init();
	imu_init();
	pressure_init();
	servo_init();
	bldc_init();
	adc_init();
	drv8301_init();
	radio_init();

	rybos_add_task(TASK_IMU_READ_PERIOD_MS, 8, (uint8_t *) "Task IMU read", task_imu_read, RYBOS_MARKER_TASK_IMU_READ, true);
	rybos_add_task(TASK_BLDC_STATUS_PERIOD_MS, 63, (uint8_t *) "Task BLDC status", task_bldc_status, RYBOS_MARKER_TASK_BLDC_STATUS, true);
	rybos_add_task(TASK_PRESSURE_READ_PERIOD_MS, 64, (uint8_t *) "Task pressure read", task_read_pressure, RYBOS_MARKER_TASK_PRESSURE_READ, true);
	rybos_add_task(TASK_RF_TIMEOUT_MS, 124, (uint8_t *) "Task RF timeout", task_rf_timeout, RYBOS_MARKER_TASK_RF_TIMEOUT, false);
	rybos_add_task(TASK_FRAME_DECODER_PERIOD_MS, 126, (uint8_t *) "Task frame decoder", task_frame_decoder, RYBOS_MARKER_TASK_FRAME_DECODER, true);
	rybos_add_task(TASK_BUZZER_PERIOD_MS, 127, (uint8_t *) "Task buzzer", task_buzzer, RYBOS_MARKER_TASK_BUZZER, false);
	rybos_add_task(TASK_LED_PERIOD_MS, 245, (uint8_t *) "Task LED status", task_led, RYBOS_MARKER_TASK_LED, true);
	rybos_add_task(TASK_LOAD_MONITOR_PERIOD_MS, 230, (uint8_t *) "Task load monitor", task_load_monitor, RYBOS_MARKER_TASK_LOAD_MONITOR, true);
	rybos_add_task(TASK_SLEEP_PERIOD_MS, 250, (uint8_t *) "Task sleep", task_sleep, RYBOS_MARKER_TASK_SLEEP, true);
	rybos_add_task(TASK_RF_PERIOD_MS, 125, (uint8_t *) "Task RF", task_rf, RYBOS_MARKER_TASK_RF, true);
	rybos_add_task(TASK_PARAM_FAST_UPDATE_PERIOD_MS, 220, (uint8_t *) "Task parameter update", task_param_update, RYBOS_MARKER_TASK_PARAM_UPDATE, true);
	rybos_add_task(TASK_LOGGER_PERIOD_MS, 240, (uint8_t *) "Task Logger", task_logger, RYBOS_MARKER_TASK_LOGGER, false);

	print_init();
	buzzer_generate_sound(BUZZER_SOUND_START);
	while (1) {
		rybos_scheduler_run();
	}
}
