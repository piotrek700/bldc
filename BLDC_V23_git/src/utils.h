#ifndef UTILS_H_
#define UTILS_H_

#include "platform.h"
#include <math.h>

//Define
#define SWAP_UINT16(x) 								((uint16_t)((x) >> 8) | (uint16_t)((x) << 8))
#define FAST_SWAP_UINT16(x) 						__REV16(x)

#define UNUSED(x) 									(void)(x)
#define IS_NAN(x)									((x) != (x))
#define UNUSED_WARNING_DISABLE 						__attribute__((unused))

//Value scaling
#define SCALE_FLOAT_TO_UINT16(value, min, max) 		(uint16_t)(((float)value - min) * 65535.0f / (max - min))
#define SCALE_UINT16_TO_FLOAT(value, min, max) 		(float)((float)value * (max - min) / 65535.0f + min)

#define SCALE_FLOAT_TO_INT16(value, min, max) 		(int16_t)(((float)value - (max + min) / 2.0f) * 65535.0f / (max - min))
#define SCALE_INT16_TO_FLOAT(value, min, max) 		(float)((float)value * (max - min) / 65535.0f + (max + min) / 2.0f)

#define RAD_TO_DEG(x)								(x * (180.0f / (float)M_PI))
#define DEG_TO_RAD(x)								(x * ((float)M_PI / 180.0f))

//Functions
float inv_sqrtf(float x);

float fast_atan2f(float y, float x);
float fast_atan2f_sec(float y, float x);
float fast_log(float val);



#endif
