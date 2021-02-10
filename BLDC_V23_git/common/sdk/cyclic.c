#include <sdk/cyclic.h>
#include <string.h>
#include <sdk/debug.h>
#include <sdk/atomic.h>

void cyclic_clear(CyclicBuffer_t *cyclic) {
	cyclic->elements = 0;
	cyclic->read_ptr = 0;
	cyclic->write_ptr = 0;
}

void cyclic_add(CyclicBuffer_t *cyclic, uint8_t *data) {
	enter_critical();

	cyclic->elements++;

	if (cyclic->elements > cyclic->length) {
		if (cyclic->overflow_allowed) {
			cyclic->elements--;

			cyclic->read_ptr += cyclic->element_size;
			if (cyclic->read_ptr >= cyclic->length * cyclic->element_size) {
				cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_BUFFER_OVERFLOW_CRITICAL);
		}
	}

	if (cyclic->elements > cyclic->max_elements) {
		cyclic->max_elements = cyclic->elements;
	}

	memcpy(cyclic->buffer + cyclic->write_ptr, data, cyclic->element_size);
	cyclic->write_ptr += cyclic->element_size;

	if (cyclic->write_ptr >= cyclic->length * cyclic->element_size) {
		cyclic->write_ptr = 0;
	}

	exit_critical();
}

bool cyclic_get(CyclicBuffer_t *cyclic, uint8_t **data) {
	enter_critical();

	if (cyclic->elements > 0) {
		*data = cyclic->buffer + cyclic->read_ptr;
		cyclic->read_ptr += cyclic->element_size;
		if (cyclic->read_ptr >= cyclic->length * cyclic->element_size) {
			cyclic->read_ptr = 0;
		}

		cyclic->elements--;

		exit_critical();
		return true;
	} else {
		exit_critical();
		return false;
	}
}

uint32_t cyclic_get_elements(CyclicBuffer_t *cyclic) {
	return cyclic->elements;
}

uint32_t cyclic_get_max_elements(CyclicBuffer_t *cyclic) {
	return cyclic->max_elements;
}

uint8_t* cyclic_get_to_add(CyclicBuffer_t *cyclic) {
	return cyclic->buffer + cyclic->write_ptr;
}

void cyclic_move(CyclicBuffer_t *cyclic) {
	cyclic->elements++;

	if (cyclic->elements > cyclic->length) {
		if (cyclic->overflow_allowed) {
			cyclic->elements--;

			cyclic->read_ptr += cyclic->element_size;
			if (cyclic->read_ptr >= cyclic->length * cyclic->element_size) {
				cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_BUFFER_OVERFLOW_CRITICAL);
		}

	}

	if (cyclic->elements > cyclic->max_elements) {
		cyclic->max_elements = cyclic->elements;
	}

	cyclic->write_ptr += cyclic->element_size;

	if (cyclic->write_ptr >= cyclic->length * cyclic->element_size) {
		cyclic->write_ptr = 0;
	}
}

