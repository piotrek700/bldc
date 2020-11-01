#ifndef PLATFORM_H_
#define PLATFORM_H_

#include "stm32f30x.h"
#include <stdbool.h>
#include <stdint.h>

#define CCMRAM_VARIABLE								__attribute__((section(".ccmram_v")))
#define CCMRAM_FUCNTION								__attribute__((section(".ccmram_f")))

#endif
