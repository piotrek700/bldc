#include <sdk/cyclic.h>
#include <string.h>
#include <sdk/debug.h>
#include <sdk/atomic.h>

void cyclic_clear(CyclicBuffer_t *p_cyclic) {
	p_cyclic->elements = 0;
	p_cyclic->read_ptr = 0;
	p_cyclic->write_ptr = 0;
}

void cyclic_add(CyclicBuffer_t *p_cyclic, uint8_t *p_data) {
	enter_critical();

	p_cyclic->elements++;

	if (p_cyclic->elements > p_cyclic->length) {
		if (p_cyclic->overflow_allowed) {
			p_cyclic->elements--;

			p_cyclic->read_ptr += p_cyclic->element_size;
			if (p_cyclic->read_ptr >= p_cyclic->length * p_cyclic->element_size) {
				p_cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_BUFFER_OVERFLOW_CRITICAL);
		}
	}

	if (p_cyclic->elements > p_cyclic->max_elements) {
		p_cyclic->max_elements = p_cyclic->elements;
	}

	memcpy(p_cyclic->p_buffer + p_cyclic->write_ptr, p_data, p_cyclic->element_size);
	p_cyclic->write_ptr += p_cyclic->element_size;

	if (p_cyclic->write_ptr >= p_cyclic->length * p_cyclic->element_size) {
		p_cyclic->write_ptr = 0;
	}

	exit_critical();
}

bool cyclic_get(CyclicBuffer_t *p_cyclic, uint8_t **pp_data) {
	enter_critical();

	if (p_cyclic->elements > 0) {
		*pp_data = p_cyclic->p_buffer + p_cyclic->read_ptr;
		p_cyclic->read_ptr += p_cyclic->element_size;
		if (p_cyclic->read_ptr >= p_cyclic->length * p_cyclic->element_size) {
			p_cyclic->read_ptr = 0;
		}

		p_cyclic->elements--;

		exit_critical();
		return true;
	} else {
		exit_critical();
		return false;
	}
}

uint32_t cyclic_get_elements(CyclicBuffer_t *p_cyclic) {
	return p_cyclic->elements;
}

uint32_t cyclic_get_max_elements(CyclicBuffer_t *p_cyclic) {
	return p_cyclic->max_elements;
}

uint8_t* cyclic_get_to_add(CyclicBuffer_t *p_cyclic) {
	return p_cyclic->p_buffer + p_cyclic->write_ptr;
}

void cyclic_move(CyclicBuffer_t *p_cyclic) {
	p_cyclic->elements++;

	if (p_cyclic->elements > p_cyclic->length) {
		if (p_cyclic->overflow_allowed) {
			p_cyclic->elements--;

			p_cyclic->read_ptr += p_cyclic->element_size;
			if (p_cyclic->read_ptr >= p_cyclic->length * p_cyclic->element_size) {
				p_cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_BUFFER_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_BUFFER_OVERFLOW_CRITICAL);
		}

	}

	if (p_cyclic->elements > p_cyclic->max_elements) {
		p_cyclic->max_elements = p_cyclic->elements;
	}

	p_cyclic->write_ptr += p_cyclic->element_size;

	if (p_cyclic->write_ptr >= p_cyclic->length * p_cyclic->element_size) {
		p_cyclic->write_ptr = 0;
	}
}

