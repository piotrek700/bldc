#ifndef CYCLIC_H_
#define CYCLIC_H_

#include <stdbool.h>
#include "../Debug/debug.h"
#include "../utils.h"
#include <string.h>

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t element_size;
	uint32_t length;
	uint32_t max_elements;
	uint8_t *buffer;
	bool overflow_allowed;
} CyclicBuffer;

#define CYCLIC_BUFFER_DEF(cyclic_name, OVERFLOW_ALLOWED, QUEUE_LENGTH, ELEMENT_SIZE)	\
static volatile uint8_t cyclic_name##_buff[QUEUE_LENGTH*ELEMENT_SIZE];					\
static volatile CyclicBuffer cyclic_name ={              					    		\
	.write_ptr = 0,                   			   										\
	.read_ptr = 0,         		  														\
	.elements	= 0,	    			   												\
	.element_size = ELEMENT_SIZE,					            						\
	.length = QUEUE_LENGTH,								                  				\
	.max_elements = 0,					                  								\
	.buffer = (uint8_t *)cyclic_name##_buff,									        \
	.overflow_allowed = OVERFLOW_ALLOWED,					          					\
};

void cyclic_clear(CyclicBuffer *cyclic);

void cyclic_add(CyclicBuffer *cyclic, uint8_t *data);

bool cyclic_get(CyclicBuffer *cyclic, uint8_t **data);

uint32_t cyclic_get_elements(CyclicBuffer *cyclic);

uint32_t cyclic_get_max_elements(CyclicBuffer *cyclic);

uint8_t* cyclic_get_to_add(CyclicBuffer *cyclic); //always call in critical section

void cyclic_move(CyclicBuffer *cyclic); //always call in critical section

#endif
