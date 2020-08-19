#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdint.h>
#include "debug_app.h"

#define DEBUG_GENERATE_ENUM(STR) 							STR,
#define DEBUG_GENERATE_CASE(STR) 							case STR:

#define debug_error(error_code)								debug_error_handler(error_code, (uint8_t *)__FILE__, __LINE__)

typedef enum {
	DEBUG_CRITICAL_ERROR_LIST(DEBUG_GENERATE_ENUM)
	DEBUG_MESSAGE_ERROR_LIST(DEBUG_GENERATE_ENUM)
	NO_ERROR = 0xFF
} DebugError;

void debug_error_handler(DebugError error, uint8_t *file, int32_t line);

#endif
