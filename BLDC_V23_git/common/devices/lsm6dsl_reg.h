#ifndef LSM6DSL_REG_H_
#define LSM6DSL_REG_H_

#define LSM6DSL_SPI_WRITE_MASK					0x00
#define LSM6DSL_SPI_READ_MASK					0x80

#define	LSM6DSL_REG_FUNC_CFG_ACCESS				0x01
#define	LSM6DSL_REG_SENSOR_SYNC_TIME_FRAME		0x04
#define	LSM6DSL_REG_SENSOR_SYNC_RES_RATIO		0x05
#define	LSM6DSL_REG_FIFO_CTRL1					0x06
#define	LSM6DSL_REG_FIFO_CTRL2					0x07
#define	LSM6DSL_REG_FIFO_CTRL3					0x08
#define	LSM6DSL_REG_FIFO_CTRL4					0x09
#define	LSM6DSL_REG_FIFO_CTRL5					0x0A
#define	LSM6DSL_REG_DRDY_PULSE_CFG_G			0x0B
#define	LSM6DSL_REG_INT1_CTRL					0x0D
#define	LSM6DSL_REG_INT2_CTRL					0x0E
#define	LSM6DSL_REG_WHO_AM_I					0x0F
#define	LSM6DSL_REG_CTRL1_XL					0x10
#define	LSM6DSL_REG_CTRL2_G						0x11
#define	LSM6DSL_REG_CTRL3_C						0x12
#define	LSM6DSL_REG_CTRL4_C						0x13
#define	LSM6DSL_REG_CTRL5_C						0x14
#define	LSM6DSL_REG_CTRL6_C						0x15
#define	LSM6DSL_REG_CTRL7_G						0x16
#define	LSM6DSL_REG_CTRL8_XL					0x17
#define	LSM6DSL_REG_CTRL9_XL					0x18
#define	LSM6DSL_REG_CTRL10_C					0x19
#define	LSM6DSL_REG_MASTER_CONFIG				0x1A
#define	LSM6DSL_REG_WAKE_UP_SRC					0x1B
#define	LSM6DSL_REG_TAP_SRC						0x1C
#define	LSM6DSL_REG_D6D_SRC						0x1D
#define	LSM6DSL_REG_STATUS_REG					0x1E
#define	LSM6DSL_REG_OUT_TEMP_L					0x20
#define	LSM6DSL_REG_OUT_TEMP_H					0x21
#define	LSM6DSL_REG_OUTX_L_G					0x22
#define	LSM6DSL_REG_OUTX_H_G					0x23
#define	LSM6DSL_REG_OUTY_L_G					0x24
#define	LSM6DSL_REG_OUTY_H_G					0x25
#define	LSM6DSL_REG_OUTZ_L_G					0x26
#define	LSM6DSL_REG_OUTZ_H_G					0x27
#define	LSM6DSL_REG_OUTX_L_XL					0x28
#define	LSM6DSL_REG_OUTX_H_XL					0x29
#define	LSM6DSL_REG_OUTY_L_XL					0x2A
#define	LSM6DSL_REG_OUTY_H_XL					0x2B
#define	LSM6DSL_REG_OUTZ_L_XL					0x2C
#define	LSM6DSL_REG_OUTZ_H_XL					0x2D
#define	LSM6DSL_REG_SENSORHUB1_REG				0x2E
#define	LSM6DSL_REG_SENSORHUB2_REG				0x2F
#define	LSM6DSL_REG_SENSORHUB3_REG				0x30
#define	LSM6DSL_REG_SENSORHUB4_REG				0x31
#define	LSM6DSL_REG_SENSORHUB5_REG				0x32
#define	LSM6DSL_REG_SENSORHUB6_REG				0x33
#define	LSM6DSL_REG_SENSORHUB7_REG				0x34
#define	LSM6DSL_REG_SENSORHUB8_REG				0x35
#define	LSM6DSL_REG_SENSORHUB9_REG				0x36
#define	LSM6DSL_REG_SENSORHUB10_REG				0x37
#define	LSM6DSL_REG_SENSORHUB11_REG				0x38
#define	LSM6DSL_REG_SENSORHUB12_REG				0x39
#define	LSM6DSL_REG_FIFO_STATUS1				0x3A
#define	LSM6DSL_REG_FIFO_STATUS2				0x3B
#define	LSM6DSL_REG_FIFO_STATUS3				0x3C
#define	LSM6DSL_REG_FIFO_STATUS4				0x3D
#define	LSM6DSL_REG_FIFO_DATA_OUT_L				0x3E
#define	LSM6DSL_REG_FIFO_DATA_OUT_H				0x3F
#define	LSM6DSL_REG_TIMESTAMP0_REG				0x40
#define	LSM6DSL_REG_TIMESTAMP1_REG				0x41
#define	LSM6DSL_REG_TIMESTAMP2_REG				0x42
#define	LSM6DSL_REG_STEP_TIMESTAMP_L			0x49
#define	LSM6DSL_REG_STEP_TIMESTAMP_H			0x4A
#define	LSM6DSL_REG_STEP_COUNTER_L				0x4B
#define	LSM6DSL_REG_STEP_COUNTER_H				0x4C
#define	LSM6DSL_REG_SENSORHUB13_REG				0x4D
#define	LSM6DSL_REG_SENSORHUB14_REG				0x4E
#define	LSM6DSL_REG_SENSORHUB15_REG				0x4F
#define	LSM6DSL_REG_SENSORHUB16_REG				0x50
#define	LSM6DSL_REG_SENSORHUB17_REG				0x51
#define	LSM6DSL_REG_SENSORHUB18_REG				0x52
#define	LSM6DSL_REG_FUNC_SRC1					0x53
#define	LSM6DSL_REG_FUNC_SRC2					0x54
#define	LSM6DSL_REG_WRIST_TILT_IA				0x55
#define	LSM6DSL_REG_TAP_CFG						0x58
#define	LSM6DSL_REG_TAP_THS_6D					0x59
#define	LSM6DSL_REG_INT_DUR2					0x5A
#define	LSM6DSL_REG_WAKE_UP_THS					0x5B
#define	LSM6DSL_REG_WAKE_UP_DUR					0x5C
#define	LSM6DSL_REG_FREE_FALL					0x5D
#define	LSM6DSL_REG_MD1_CFG						0x5E
#define	LSM6DSL_REG_MD2_CFG						0x5F
#define	LSM6DSL_REG_MASTER_CMD_CODE				0x60
#define	LSM6DSL_REG_SENS_SYNC_SPI_ERROR_CODE	0x61
#define	LSM6DSL_REG_OUT_MAG_RAW_X_L				0x66
#define	LSM6DSL_REG_OUT_MAG_RAW_X_H				0x67
#define	LSM6DSL_REG_OUT_MAG_RAW_Y_L				0x68
#define	LSM6DSL_REG_OUT_MAG_RAW_Y_H				0x69
#define	LSM6DSL_REG_OUT_MAG_RAW_Z_L				0x6A
#define	LSM6DSL_REG_OUT_MAG_RAW_Z_H				0x6B
#define	LSM6DSL_REG_X_OFS_USR					0x73
#define	LSM6DSL_REG_Y_OFS_USR					0x74
#define	LSM6DSL_REG_Z_OFS_USR					0x75

#define LSM6DSL_WHO_AM_I_RESPONSE				0x6A

#define LSM6DSL_CTRL1_XL_8G_833HZ				0x7C
#define LSM6DSL_CTRL2_G_2000DPS_833HZ			0x7C

#define LSM6DSL_CTRL1_XL_8G_1660HZ				0x8C
#define LSM6DSL_CTRL2_G_2000DPS_1660HZ			0x8C

#define LSM6DSL_CTRL1_XL_8G_6660HZ				0xAC
#define LSM6DSL_CTRL2_G_2000DPS_6660HZ			0xAC

#define LSM6DSL_CTRL4_C_LPF1_SEL_G_EN			0x02

#define LSM6DSL_CTRL6_C_FTYPE_315HZ				0x00
#define LSM6DSL_CTRL6_C_FTYPE_273HZ				0x01
#define LSM6DSL_CTRL6_C_FTYPE_173HZ				0x02
#define LSM6DSL_CTRL6_C_FTYPE_937HZ				0x03

#define LSM6DSL_CTRL8_XL_LPF2_XL_EN_ODR50		0x88		//133,2Hz	6600Hz
#define LSM6DSL_CTRL8_XL_LPF2_XL_EN_ODR100		0xA8		//66,6Hz	6600Hz
#define LSM6DSL_CTRL8_XL_LPF2_XL_EN_ODR9		0xD8		//740Hz		6600Hz
#define LSM6DSL_CTRL8_XL_LPF2_XL_EN_ODR400		0xE8		//16,65Hz	6600Hz

#endif
