#ifndef UTILS_H_
#define UTILS_H_

#include "platform.h"
#include <math.h>

//Define
#define SWAP_UINT16(x) 								((uint16_t)((x) >> 8) | (uint16_t)((x) << 8))	//TODO instruction in arm
#define UNUSED(x) 									(void)(x)
#define IS_NAN(x)									((x) != (x))
#define UNUSED_WARNING_DISABLE 						__attribute__((unused))

#define CCMRAM_VARIABLE								__attribute__((section(".ccmram_v")))
#define CCMRAM_FUCNTION								__attribute__((section(".ccmram_f")))

//Functions
float inv_sqrtf(float x);

float fast_atan2f(float y, float x);
float fast_atan2f_sec(float y, float x);
float fast_log(float val);

void safe_increment(uint32_t *addr);
void safe_decrement(uint32_t *addr);

void enter_critical(void);
void exit_critical(void);

uint32_t critiacl_get_max_queue_depth(void);

#endif
