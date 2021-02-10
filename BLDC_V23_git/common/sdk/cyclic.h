#ifndef CYCLIC_H_
#define CYCLIC_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t element_size;
	uint32_t length;
	uint32_t max_elements;
	uint8_t *buffer;
	bool overflow_allowed;
} CyclicBuffer_t;

#define CYCLIC_BUFFER_DEF(_cyclic_name, _overflow_allowed, _queue_length, _element_size)\
static volatile uint8_t _cyclic_name##_buff[_queue_length*_element_size];				\
static volatile CyclicBuffer_t _cyclic_name ={              					    	\
	.write_ptr = 0,                   			   										\
	.read_ptr = 0,         		  														\
	.elements	= 0,	    			   												\
	.element_size = _element_size,					            						\
	.length = _queue_length,								                  			\
	.max_elements = 0,					                  								\
	.buffer = (uint8_t *)_cyclic_name##_buff,									        \
	.overflow_allowed = _overflow_allowed,					          					\
};

void cyclic_clear(CyclicBuffer_t *cyclic);

void cyclic_add(CyclicBuffer_t *cyclic, uint8_t *data);

bool cyclic_get(CyclicBuffer_t *cyclic, uint8_t **data);

uint32_t cyclic_get_elements(CyclicBuffer_t *cyclic);

uint32_t cyclic_get_max_elements(CyclicBuffer_t *cyclic);

uint8_t* cyclic_get_to_add(CyclicBuffer_t *cyclic); //always call in critical section

void cyclic_move(CyclicBuffer_t *cyclic); //always call in critical section

#endif
