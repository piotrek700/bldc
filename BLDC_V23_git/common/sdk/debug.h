#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>
#include <settings/debug_error_list.h>
#include "Debug/debug_app.h"

#define DEBUG_GENERATE_ENUM(str) 							str,
#define DEBUG_GENERATE_CASE(str) 							case str:
#define DEBUG_STRING(str) 									#str,

#define debug_error(error_code)								debug_error_handler(error_code, (uint8_t *)__FILE__, __LINE__)

typedef enum {
	DEBUG_CRITICAL_ERROR_LIST(DEBUG_GENERATE_ENUM)
	DEBUG_CRITICAL_NOT_CRITICAL_PTR,
	DEBUG_MESSAGE_ERROR_LIST(DEBUG_GENERATE_ENUM)
} DebugError_t;

const char * debug_get_error_string(DebugError_t error);

void debug_error_handler(DebugError_t error, uint8_t *p_file, int32_t line);

DebugError_t *debug_get_last_error(void);

#endif
