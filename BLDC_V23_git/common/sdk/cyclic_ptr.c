#include <sdk/cyclic_ptr.h>
#include <sdk/debug.h>
#include <sdk/atomic.h>

void cyclic_ptr_clear(CyclicPtrBuffer_t *cyclic) {
	cyclic->elements = 0;
	cyclic->read_ptr = 0;
	cyclic->write_ptr = 0;
}

void cyclic_ptr_add(CyclicPtrBuffer_t *cyclic, BuffType_t data) {
	enter_critical();

	cyclic->elements++;

	if (cyclic->elements > cyclic->length) {
		if (cyclic->overflow_allowed) {
			cyclic->elements--;

			cyclic->read_ptr++;
			if (cyclic->read_ptr >= cyclic->length) {
				cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_PTR_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_PTR_OVERFLOW_CRITICAL);
		}
	}

	if (cyclic->elements > cyclic->max_elements) {
		cyclic->max_elements = cyclic->elements;
	}

	cyclic->buffer[cyclic->write_ptr] = data;
	cyclic->write_ptr++;
	if (cyclic->write_ptr == cyclic->length) {
		cyclic->write_ptr = 0;
	}

	exit_critical();
}

bool cyclic_ptr_get(CyclicPtrBuffer_t *cyclic, BuffType_t *data) {
	enter_critical();
	if (cyclic->elements > 0) {
		*data = cyclic->buffer[cyclic->read_ptr];
		cyclic->read_ptr++;
		if (cyclic->read_ptr == cyclic->length) {
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

uint32_t cyclic_ptr_get_elements(CyclicPtrBuffer_t *cyclic) {
	return cyclic->elements;
}

uint32_t cyclic_ptr_get_max_elements(CyclicPtrBuffer_t *cyclic) {
	return cyclic->max_elements;
}

