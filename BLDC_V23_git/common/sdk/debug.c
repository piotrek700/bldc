#include <sdk/debug.h>
#include <sdk/utils.h>

static const char *debug_error_string[] = {
		DEBUG_CRITICAL_ERROR_LIST(DEBUG_STRING)
		"DEBUG_CRITICAL_NOT_CRITICAL_PTR",
		DEBUG_MESSAGE_ERROR_LIST(DEBUG_STRING)
};

static DebugError_t last_error[DEBUG_ERROR_HISTORY_LEN];

void __attribute__((weak)) debug_critical_error(DebugError_t error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);

	while (1) {
		__NOP();
	}
}

void __attribute__((weak)) debug_message_error(DebugError_t error, uint8_t *file, int32_t line) {
	UNUSED(error);
	UNUSED(file);
	UNUSED(line);
}

const char * debug_get_error_string(DebugError_t error) {
	return debug_error_string[error];
}

void debug_error_handler(DebugError_t error, uint8_t *file, int32_t line) {
	//Update error history
	uint32_t i;
	for (i = DEBUG_ERROR_HISTORY_LEN - 1; i > 0; i--) {
		last_error[i] = last_error[i - 1];
	}
	last_error[0] = error;

	if (error < DEBUG_CRITICAL_NOT_CRITICAL_PTR) {
		debug_critical_error(error, file, line);
		return;
	}

	if (error > DEBUG_CRITICAL_NOT_CRITICAL_PTR) {
		debug_message_error(error, file, line);
		return;
	}
}

DebugError_t *debug_get_last_error(void) {
	return last_error;
}
