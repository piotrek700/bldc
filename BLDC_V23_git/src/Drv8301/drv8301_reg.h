#ifndef DRV8301_REG_H_
#define DRV8301_REG_H_

#define DRV8301_SPI_READ							0x8000
#define DRV8301_SPI_WRITE							0x0000
#define DRV8301_DATA_MASK							0x07FF
#define DRV8301_ADDRESS_MASK						0x7800

#define DRV8301_ADDR0_SR1							0x0000
#define DRV8301_ADDR1_SR2							0x0800
#define DRV8301_ADDR2_CR1							0x1000
#define DRV8301_ADDR3_CR2							0x1800

#define DRV8301_READ_FAULT							0x8000
#define DRV8301_DATA_MASK							0x07FF

#define DRV8301_SR1_FAULT_MASK						0x07FF
#define DRV8301_SR2_FAULT_MASK						0x0100

//Mask of the FAULT bits
#define DRV8301_SR1_FAULT							0x0040
//Mask of the GVDD_UV (DRV8301 Vdd, Under Voltage) bits
#define DRV8301_SR1_GVDD_UV							0x0020
//Mask of the PVDD_UV (Power supply Vdd, Under Voltage) bits
#define DRV8301_SR1_PVDD_UV							0x0010
//Mask of the OTSD (Over Temperature Shut Down) bits
#define DRV8301_SR1_OTSD							0x0080
//Mask of the OTW (Over Temperature Warning) bits
#define DRV8301_SR1_OTW								0x0040
//Mask of the FETHA_OC (FET High side, Phase A Over Current) bits
#define DRV8301_SR1_FETHA_OC						0x0020
//Mask of the FETLA_OC (FET Low side, Phase A Over Current) bits
#define DRV8301_SR1_FETLA_OC						0x0010
//Mask of the FETHB_OC (FET High side, Phase B Over Current) bits
#define DRV8301_SR1_FETHB_OC						0x0008
//Mask of the FETLB_OC (FET Low side, Phase B Over Current) bits
#define DRV8301_SR1_FETLB_OC						0x0004
//Mask of the FETHC_OC (FET High side, Phase C Over Current) bits
#define DRV8301_SR1_FETHC_OC						0x0002
//Mask of the FETLC_OC (FET Low side, Phase C Over Current) bits
#define DRV8301_SR1_FETLC_OC						0x0001

//Mask of the Device ID bits
#define DRV8301_SR2_Device ID						0x000F
//Mask of the GVDD_OV (DRV8301 Vdd, Over Voltage) bits
#define DRV8301_SR2_GVDD_OV							0x0100

#define DRV8301_CR1_GATE_CURRENT					0x0003
#define DRV8301_CR1_GATE_CURRENT_1P7A				0x0000
#define DRV8301_CR1_GATE_CURRENT_0P7A				0x0001
#define DRV8301_CR1_GATE_CURRENT_0P25A				0x0002

#define DRV8301_CR1_GATE_RESET						0x0004
#define DRV8301_CR1_GATE_RESET_NORMAL_MODE			0x0000
#define DRV8301_CR1_GATE_RESET_RESET_GDL_FAULTS		0x0004

#define DRV8301_CR1_PWM_MODE						0x0008
#define DRV8301_CR1_PWM_MODE_6PWM					0x0000
#define DRV8301_CR1_PWM_MODE_3PWM					0x0008

#define DRV8301_CR1_OCP_MODE						0x0040
#define DRV8301_CR1_OCP_MODE_CURRENT_LIMIT			0x0000
#define DRV8301_CR1_OCP_MODE_OC_LATCH_SHUT_DOWN		0x0010
#define DRV8301_CR1_OCP_MODE_REPORT_ONLY			0x0020
#define DRV8301_CR1_OCP_MODE_OC_DISABLE				0x0040

//Overcurrent Trip = OC_ADJ_SET / MOSFET RDS(on) (5)
//NTMFS5C604NL RDS ON 10V  = TYP 0.93mOhm
//NTMFS5C604NL RDS ON 4.5V = TYP 1.25mOHm

#define DRV8301_CR1_OC_ADJ_SET						0xF800
#define DRV8301_CR1_OC_ADJ_SET_0P060				(0<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P068				(1<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P076				(2<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P086				(3<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P097				(4<<6) //104 A
#define DRV8301_CR1_OC_ADJ_SET_0P109				(5<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P123				(6<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P138				(7<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P155				(8<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P175				(9<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P197				(10<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P222				(11<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P250				(12<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P282				(13<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P317				(14<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P358				(15<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P403				(16<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P454				(17<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P511				(18<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P576				(19<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P648				(20<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P730				(21<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P822				(22<<6)
#define DRV8301_CR1_OC_ADJ_SET_0P926				(23<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P043				(24<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P175				(25<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P324				(26<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P491				(27<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P679				(28<<6)
#define DRV8301_CR1_OC_ADJ_SET_1P892				(29<<6)
#define DRV8301_CR1_OC_ADJ_SET_2P131				(30<<6)
#define DRV8301_CR1_OC_ADJ_SET_2P400				(31<<6)

#define DRV8301_CR2_OCTW_MODE						0x0003
#define DRV8301_CR2_OCTW_MODE_OT_AND_OC				0x0000
#define DRV8301_CR2_OCTW_MODE_OT_ONLY				0x0001
#define DRV8301_CR2_OCTW_MODE_OC_ONLY				0x0002

#define DRV8301_CR2_GAIN							0x000C
#define DRV8301_CR2_GAIN_10V						0x0000
#define DRV8301_CR2_GAIN_20V						0x0004
#define DRV8301_CR2_GAIN_40V						0x0008
#define DRV8301_CR2_GAIN_80V						0x000C

#define DRV8301_CR2_DC_CAL_CH1						0x0010
#define DRV8301_CR2_DC_CAL_CH1_CONNECT				0x0000
#define DRV8301_CR2_DC_CAL_CH1_DISCONNECT			0x0010

#define DRV8301_CR2_DC_CAL_CH2						0x0020
#define DRV8301_CR2_DC_CAL_CH2_CONNECT				0x0000
#define DRV8301_CR2_DC_CAL_CH2_DISCONNECT			0x0020

#define DRV8301_CR2_OC_TOFF							0x0040
#define DRV8301_CR2_OC_TOFF_CYCLE_By_CYCLE			0x0000
#define DRV8301_CR2_OC_TOFF_OFF_TIME				0x0040


/*
typedef enum{
 DRV8301_ADDR0_SR1 = 0b0000,
 DRV8301_ADDR1_SR2 = 0b0001,
 DRV8301_ADDR2_CR1 = 0b0010,
 DRV8301_ADDR3_CR2 = 0b0011
}Drv8301_addr;
*/

/*

typedef struct{
	uint16_t 				:5;
	uint16_t fault			:1;
	uint16_t fgvdd_uv		:1;
	uint16_t fpvdd_uv		:1;
	uint16_t fotsd			:1;
	uint16_t fotw			:1;
	uint16_t ffetha_oc		:1;
	uint16_t ffetla_oc		:1;
	uint16_t ffethb_oc		:1;
	uint16_t ffetlb_oc		:1;
	uint16_t ffethc_oc		:1;
	uint16_t ffetlc_oc		:1;
}Drv8301_reg_sr1;

typedef struct{
	uint16_t 				:5;
	uint16_t 				:3;
	uint16_t gvdd_Ov 		:1;
	uint16_t 				:3;
	uint16_t device_id 		:4;
}Drv8301_reg_sr2;

typedef struct{
	uint16_t 				:5;
	uint16_t oc_adj_set		:5;
	uint16_t ocp_mode		:2;
	uint16_t pwm_mode		:1;
	uint16_t gate_reset		:1;
	uint16_t gate_current	:2;
}Drv8301_reg_cr1;

typedef struct{
	uint16_t 				:5;
	uint16_t 				:4;
	uint16_t oc_toff		:1;
	uint16_t dc_cal_ch2		:1;
	uint16_t dc_cal_ch1		:1;
	uint16_t gain			:2;
	uint16_t octw_mode		:2;
}Drv8301_reg_cr2;

typedef union{
	Drv8301_reg_sr1 sr1;
	Drv8301_reg_sr2 sr2;
	Drv8301_reg_cr1 cr1;
	Drv8301_reg_cr2 cr2;
	uint16_t raw;
}Drv8301_reg;

typedef struct{
	uint16_t rw				:1;
	uint16_t addr			:4;
	uint16_t data			:11;
}Drv8301_spi_data;

*/
#endif
