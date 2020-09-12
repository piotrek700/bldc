#include "debug.h"
#include "utils.h"

static const char *debug_error_string[] = {
		DEBUG_CRITICAL_ERROR_LIST(DEBUG_STRING)
		"DEBUG_CRITICAL_NOT_CRITICAL_PTR",
		DEBUG_MESSAGE_ERROR_LIST(DEBUG_STRING)
};

void __attribute__((weak)) debug_critical_error(DebugError error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);

	while (1) {
		__NOP();
	}
}

void __attribute__((weak)) debug_message_error(DebugError error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);
}

const char * debug_get_error_string(DebugError error) {
	return debug_error_string[error];
}

void debug_error_handler(DebugError error, uint8_t *file, int32_t line) {
	if(error < DEBUG_CRITICAL_NOT_CRITICAL_PTR){
		debug_critical_error(error, file, line);
		return;
	}

	if(error > DEBUG_CRITICAL_NOT_CRITICAL_PTR){
		debug_message_error(error, file, line);
		return;
	}
}

