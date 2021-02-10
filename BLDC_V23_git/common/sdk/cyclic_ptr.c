#include <sdk/cyclic_ptr.h>
#include <sdk/debug.h>
#include <sdk/atomic.h>

void cyclic_ptr_clear(CyclicPtrBuffer_t *p_cyclic) {
	p_cyclic->elements = 0;
	p_cyclic->read_ptr = 0;
	p_cyclic->write_ptr = 0;
}

void cyclic_ptr_add(CyclicPtrBuffer_t *p_cyclic, BuffType_t data) {
	enter_critical();

	p_cyclic->elements++;

	if (p_cyclic->elements > p_cyclic->length) {
		if (p_cyclic->overflow_allowed) {
			p_cyclic->elements--;

			p_cyclic->read_ptr++;
			if (p_cyclic->read_ptr >= p_cyclic->length) {
				p_cyclic->read_ptr = 0;
			}

			debug_error(CYCLIC_PTR_OVERFLOW_NO_CRITICAL);
		} else {
			debug_error(CYCLIC_PTR_OVERFLOW_CRITICAL);
		}
	}

	if (p_cyclic->elements > p_cyclic->max_elements) {
		p_cyclic->max_elements = p_cyclic->elements;
	}

	p_cyclic->p_buffer[p_cyclic->write_ptr] = data;
	p_cyclic->write_ptr++;
	if (p_cyclic->write_ptr == p_cyclic->length) {
		p_cyclic->write_ptr = 0;
	}

	exit_critical();
}

bool cyclic_ptr_get(CyclicPtrBuffer_t *p_cyclic, BuffType_t *p_data) {
	enter_critical();
	if (p_cyclic->elements > 0) {
		*p_data = p_cyclic->p_buffer[p_cyclic->read_ptr];
		p_cyclic->read_ptr++;
		if (p_cyclic->read_ptr == p_cyclic->length) {
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

uint32_t cyclic_ptr_get_elements(CyclicPtrBuffer_t *p_cyclic) {
	return p_cyclic->elements;
}

uint32_t cyclic_ptr_get_max_elements(CyclicPtrBuffer_t *p_cyclic) {
	return p_cyclic->max_elements;
}

