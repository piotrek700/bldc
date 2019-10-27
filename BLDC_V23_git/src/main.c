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

static void task_rf_timeout(void) {
	radio_timeout();
}

static void task_sleep(void) {
	//TODO implement all sleep functions
	__WFI();
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

static void rc_disconnected(void){
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
		height_offset = height_tmp / (float) (offset_cnt - 1);
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

	float tmp;
	tmp = (float)rc_yaw * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_2_TOP, tmp);

	tmp = (float)rc_pitch * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_4_BOTTOM, tmp);

	tmp = (float)rc_roll * 45.0f/2048.0f;
	servo_set_position_angle(SERVO_POSITION_1_LEFT, tmp);

	tmp = (float)rc_throttle * 45.0f/2048.0f;
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
		vel_offset[0] = vel_offset_tmp[0] / (float) (offset_cnt - 1);
		vel_offset[1] = vel_offset_tmp[1] / (float) (offset_cnt - 1);
		vel_offset[2] = vel_offset_tmp[2] / (float) (offset_cnt - 1);
		offset_cnt++;
		ahrs_set_beta(AHRS_BETA_FINAL);
	}

	vel_compensated[0] = vel[0] - vel_offset[0];
	vel_compensated[1] = vel[1] - vel_offset[1];
	vel_compensated[2] = vel[2] - vel_offset[2];

	//AHRS update
	ahrs_update(vel_compensated[0] * (float) M_PI / 180.0f, -vel_compensated[1] * (float) M_PI / 180.0f, -vel_compensated[2] * (float) M_PI / 180.0f, //YPR 0 0 0
	acc[0], -acc[1], -acc[2]);

	ahrs_rotate_45();

	yaw = ahrs_get_yaw();
	pitch = ahrs_get_pitch();
	roll = ahrs_get_roll();

	imu_read_sensor();

	control_drone();
}

void frame_cb_frame_req_init_data(void *buff, uint8_t params) {
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
	frame_radio_send(FRAME_TYPE_RESP_INIT_DATA, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_fast_param(void) {
	FrameFastParamsSlave frame;

	float *q = ahrs_get_q2();
	float *angle = servo_get_angle();

	frame.ahrs.q[0] = q[0] * 32767.0f;																		//	  -1 	-     1
	frame.ahrs.q[1] = q[1] * 32767.0f;																		//	  -1 	-     1
	frame.ahrs.q[2] = q[2] * 32767.0f;																		//	  -1 	-     1
	frame.ahrs.q[3] = q[3] * 32767.0f;																		//	  -1 	-     1

	frame.servo.angle[0] = (angle[0] * 32767.0f) / 180.0f;													//	-180 	-  	180		deg
	frame.servo.angle[1] = (angle[1] * 32767.0f) / 180.0f;													//	-180 	-  	180		deg
	frame.servo.angle[2] = (angle[2] * 32767.0f) / 180.0f;													//	-180 	-  	180		deg
	frame.servo.angle[3] = (angle[3] * 32767.0f) / 180.0f;													//	-180 	-  	180		deg

	//Slave->Master->PC
	frame_radio_send(FRAME_TYPE_FAST_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_slow_param(void) {
	FrameSlowParamSlave frame;

	//ADC
	frame.adc.ntc_temp = ((adc_get_ntc_temperature_c() + 128.0f) * 65535.0f) / (128.0f + 256.0f);			//	-128 	- 	256		C
	frame.adc.up_temp = ((adc_get_up_temperature_c() + 128.0f) * 65535.0f) / (128.0f + 256.0f);				//	-128 	- 	256		C
	frame.adc.bat_v = (adc_get_v_vcc_v() * 65535.0f) / 32.0f;												//	   0 	-  	 32 	V
	frame.adc.ldo_v = (adc_get_v_ldo_v() * 65535.0f) / 4.0f;												//	   0 	-     4		V

	//IMU
	float *acc = imu_get_imu_acceleration();

	frame.imu.acc[0] = (acc[0] * 32767.0f) / 80.0f;															//	 -80 	-    80		m/s2
	frame.imu.acc[1] = (acc[1] * 32767.0f) / 80.0f;															//	 -80 	-    80		m/s2
	frame.imu.acc[2] = (acc[2] * 32767.0f) / 80.0f;															//	 -80 	-    80		m/s2

	frame.imu.vel[0] = (vel_compensated[0] * 32767.0f) / 2000.0f;											//   -2k 	-  	 2k		deg/s
	frame.imu.vel[1] = (vel_compensated[1] * 32767.0f) / 2000.0f;											//   -2k 	-  	 2k		deg/s
	frame.imu.vel[2] = (vel_compensated[2] * 32767.0f) / 2000.0f;											//   -2k 	-  	 2k		deg/s

	frame.imu.temp = ((imu_get_temperature_c() + 128.0f) * 65535.0f) / (128.0f + 256.0f);					//	-128 	- 	256		C

	//Pressure
	frame.pressure.height = (height_compensated * 32767.0f) / 1000.0f;										//	 -1000 	-  1000		m
	frame.pressure.press = ((pressure_get_pressure_pa() - 90000.0f) * 65535.0f) / (110000.0f - 90000.0f);	//	 90k	-  110k 	Pa
	frame.pressure.temp = ((pressure_get_temperature_c() + 128.0f) * 65535.0f) / (128.0f + 256.0f);			//	-128 	- 	256		C

	//Slave->Master->PC
	frame_radio_send(FRAME_TYPE_SLOW_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void undervoltage_protection(void) {
	static uint32_t time = 0;

	if (adc_get_v_vcc_v() < ADC_MIN_BAT_V) {
		if (tick_get_time_ms() - time > UNDEVOLTAGE_SOUND_PERIOD_MS) {
			time = tick_get_time_ms();
			buzzer_generate_sound(BUZZER_SOUND_SINGLE_PEAK);
		}
	}
}

static void overtemperature_protection(void) {
	static uint32_t time = 0;

	if (adc_get_ntc_temperature_c() > ADC_NTC_MAX_TEMP_C) {
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
	frame_uart_send(FRAME_TYPE_REQ_DISPLAY_CHANNELS, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);

	//TODO send KP KI KD to master and pc
}

static void print_cpu_load(void) {
	FrameSystemLoadSlave frame;

	uint32_t i;
	float accumulate = 0;
	float task_load;
	float cnt;

	printf("------------------------------------\n");
	printf("------------Load Monitor------------\n");
	printf("ID\tLoad[%%]\tExecute Cnt\tTask name\n");
	printf("------------------------------------\n");

	for (i = 0; i < MARKER_SYSTEM; i++) {
		task_load = rybos_get_task_execution_time(i) / (TICK_CPU_FREQUENCY_HZ / 1000000);
		rybos_clear_task_execution_time(i);

		task_load /= 10000.0f * (float) TASK_LOAD_MONITOR_PERIOD_S;
		frame.load[i] = (uint16_t) (task_load * 65535.0f / 100.0f);	//Scale 0 - 65535

		accumulate += task_load;

		cnt = (float) rybos_get_task_execution_cnt(i) / (float) TASK_LOAD_MONITOR_PERIOD_S;
		rybos_clear_task_execution_cnt(i);

		printf("%u\t%6.3f\t%9.2f\t%s\n", (unsigned int) i + 1, task_load, cnt, TASK_NAMES[i]);

	}
	task_load = 100.0f - accumulate;
	frame.load[i] = (uint16_t) (task_load * 65535.0f / 100.0f);	//Scale 0 - 65535

	cnt = (float) rybos_get_task_execution_cnt(i) / (float) TASK_LOAD_MONITOR_PERIOD_S;
	rybos_clear_task_execution_cnt(i);

	printf("%u\t%6.3f\t%9.2f\t%s\n", (unsigned int) i + 1, task_load, cnt, TASK_NAMES[i]);

	//TODO not elegant - print the BLDC left time
	//frame.load[11] = adc_bldc_left_time() * 65535.0f / 100.0f;	//TODO remove

	//Slave->Master->PC
	frame_radio_send(FRAME_TYPE_SYSTEM_LOAD_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_radio_parameters(void) {
	FrameRadioStat frame;

	frame.avarage_rssi = ((radio_get_avarage_rssi() + 256.0f) * 65535.0f) / (256.0f + 32.0f);	//Scale -256 - 32
	frame.max_rssi = (((float) radio_get_max_rssi() + 256.0f) * 65535.0f) / (256.0f + 32.0f);	//Scale -256 - 32
	frame.min_rssi = (((float) radio_get_min_rssi() + 256.0f) * 65535.0f) / (256.0f + 32.0f);	//Scale -256 - 32

	frame.max_tran_deph = radio_get_max_queue_depth();
	frame.rettransmition = radio_get_retransmition_cnt();
	frame.rx_bytes = radio_get_received_bytes();
	frame.tx_bytes = radio_get_transmited_bytes();
	frame.recived_frame = radio_get_received_frame();
	frame.recived_frame_total = radio_get_received_frame_total();
	frame.transmitted_frame = radio_get_transmitted_frame();

	radio_reset_statistics();

	//Slave->Master->PC
	frame_radio_send(FRAME_TYPE_RADIO_STAT, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void print_system_param(void) {
	FrameSystemParamSlave frame;

	frame.critical_deh = critiacl_get_max_queue_depth();
	frame.spi_tran_deph = spi_get_max_queue_depth();
	frame.system_local_time = tick_get_time_ms();

	//Slave->Master->PC
	frame_radio_send(FRAME_TYPE_SYSTEM_PARAMS_SLAVE, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
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
	frame_radio_send(FRAME_TYPE_UART_STAT, (uint8_t *) (&frame), FRAME_SOURCE_SLAVE | FRAME_DESTINATION_MASTER_PC);
}

static void task_load_monitor(void) {
	print_cpu_load();
	print_radio_parameters();
	print_system_param();
	print_uart_param();
}


void frame_cb_frame_rc_control(void *buff, uint8_t params) {
	UNUSED(params);

	FrameRcControl *frame = (FrameRcControl *) buff;

	if(rc_connected){
		rc_yaw = frame->yaw;
		rc_pitch = frame->pitch;
		rc_roll = frame->roll;
		rc_throttle = frame->throttle;
		rc_status = frame->status;
	}else{
		rc_disconnected();
	}
}

/*
 * ####################################################################################################################
 * Name		| IRQ		| P	| E | Peripherals 	| Finished 	| To do
 * ####################################################################################################################
 * ADC		|			| 4	|	|				|	-		| ????
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
 * Tick		| SysTick	| 1	| -	| SystTick		|	+		| Replace by 32 bit timer - tim2 - not possible in this hardware(PA0<->PA1)
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
 */

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
	adc_init();
	adc_set_bldc_enable(true);
	drv8301_init();
	radio_init();

	rybos_add_task(TASK_IMU_READ_PERIOD_MS, 			8, 		(uint8_t *) "Task IMU read", 			task_imu_read, 			MARKER_TASK_IMU_READ, 		true);
	rybos_add_task(TASK_BLDC_STATUS_PERIOD_MS, 			63, 	(uint8_t *) "Task BLDC status", 		task_bldc_status, 		MARKER_TASK_BLDC_STATUS, 	true);
	rybos_add_task(TASK_PRESSURE_READ_PERIOD_MS, 		64, 	(uint8_t *) "Task pressure read", 		task_read_pressure, 	MARKER_TASK_PRESSURE_READ, 	true);
	rybos_add_task(TASK_RF_TIMEOUT_MS, 					124, 	(uint8_t *) "Task RF timeout", 			task_rf_timeout, 		MARKER_TASK_RF_TIMEOUT, 	false);
	rybos_add_task(TASK_FRAME_DECODER_PERIOD_MS, 		126, 	(uint8_t *) "Task frame decoder", 		task_frame_decoder, 	MARKER_TASK_FRAME_DECODER, 	true);
	rybos_add_task(TASK_BUZZER_PERIOD_MS, 				127, 	(uint8_t *) "Task buzzer", 				task_buzzer, 			MARKER_TASK_BUZZER, 		false);
	rybos_add_task(TASK_LED_PERIOD_MS, 					245, 	(uint8_t *) "Task LED status", 			task_led, 				MARKER_TASK_LED, 			true);
	rybos_add_task(TASK_LOAD_MONITOR_PERIOD_MS, 		230, 	(uint8_t *) "Task load monitor", 		task_load_monitor, 		MARKER_TASK_LOAD_MONITOR, 	true);
	rybos_add_task(TASK_SLEEP_PERIOD_MS, 				250, 	(uint8_t *) "Task sleep", 				task_sleep, 			MARKER_TASK_SLEEP, 			true);
	rybos_add_task(TASK_RF_PERIOD_MS, 					125, 	(uint8_t *) "Task RF", 					task_rf, 				MARKER_TASK_RF, 			true);
	rybos_add_task(TASK_PARAM_FAST_UPDATE_PERIOD_MS, 	220, 	(uint8_t *) "Task parameter update", 	task_param_update, 		MARKER_TASK_PARAM_UPDATE, 	true);
	rybos_add_task(TASK_LOGGER_PERIOD_MS, 				240, 	(uint8_t *) "Task Logger", 				task_logger, 			MARKER_TASK_LOGGER, 		false);

	print_init();
	buzzer_generate_sound(BUZZER_SOUND_START);

	while (1) {
		rybos_scheduler_run();
	}
}
