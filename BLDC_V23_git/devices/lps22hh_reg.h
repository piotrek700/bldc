#ifndef LPS22HH_REG_H_
#define LPS22HH_REG_H_

#define LPS22HH_SPI_WRITE_MASK				0x00
#define LPS22HH_SPI_READ_MASK				0x80

#define LPS22HH_REG_INTERRUPT_CFG 			0x0B
#define LPS22HH_REG_THS_P_L 				0x0C
#define LPS22HH_REG_THS_P_H 				0x0D
#define LPS22HH_REG_IF_CTRL 				0x0E
#define LPS22HH_REG_WHO_AM_I 				0x0F
#define LPS22HH_REG_CTRL_REG1 				0x10
#define LPS22HH_REG_CTRL_REG2 				0x11
#define LPS22HH_REG_CTRL_REG3 				0x12
#define LPS22HH_REG_FIFO_CTRL 				0x13
#define LPS22HH_REG_FIFO_WTM 				0x14
#define LPS22HH_REG_REF_P_L 				0x15
#define LPS22HH_REG_REF_P_H 				0x16
#define LPS22HH_REG_RPDS_L 					0x18
#define LPS22HH_REG_RPDS_H 					0x19
#define LPS22HH_REG_INT_SOURCE 				0x24
#define LPS22HH_REG_FIFO_STATUS1 			0x25
#define LPS22HH_REG_FIFO_STATUS2 			0x26
#define LPS22HH_REG_STATUS 					0x27
#define LPS22HH_REG_PRESSURE_OUT_XL 		0x28
#define LPS22HH_REG_PRESSURE_OUT_L 			0x29
#define LPS22HH_REG_PRESSURE_OUT_H 			0x2A
#define LPS22HH_REG_TEMP_OUT_L 				0x2B
#define LPS22HH_REG_TEMP_OUT_H 				0x2C
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_XL 	0x78
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_L 	0x79
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_H 	0x7A
#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_L 	0x7B
#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_H 	0x7C

#define LPS22HH_WHO_AM_I_RESPONSE			0xB3
#define LPS22HH_CTRL_REG1_P_T_200HZ 		0xE0

#endif