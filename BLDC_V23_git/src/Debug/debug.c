#include "debug.h"
#include "utils.h"

void __attribute__((weak)) debug_critical_error(DebugError error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);
	while (1){
		__NOP();
	}
}

void __attribute__((weak)) debug_message_error(DebugError error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);
}

void debug_error_handler(DebugError error, uint8_t *file, int32_t line){
	switch (error) {
		//case ALL_CRITIAL:
		DEBUG_CRITICAL_ERROR_LIST(DEBUG_GENERATE_CASE)
		debug_critical_error(error, file, line);
		break;

		//case ALL_MESSAGE
		DEBUG_MESSAGE_ERROR_LIST(DEBUG_GENERATE_CASE)
		debug_message_error(error, file, line);
		break;

		//No error do nothing
		case NO_ERROR:
		break;

		default:
		debug_error(DEBUG_ERROR_NOT_HANDLE);
		break;
	}
}

