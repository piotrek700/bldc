#ifndef CYCLIC_BYTE_H_
#define CYCLIC_BYTE_H_

#include <stdbool.h>
#include "../Debug/debug.h"
#include "../utils.h"

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t length;
	uint32_t max_elements;
	uint8_t *buffer;
	bool overflow_allowed;
} CyclicByteBuffer;

#define CYCLIC_BUFFER_BYTE_DEF(cyclic_name, OVERFLOW_ALLOWED, QUEUE_LENGTH)	\
static volatile uint8_t cyclic_name##_buff[QUEUE_LENGTH];					\
static volatile CyclicByteBuffer cyclic_name ={              				\
	.write_ptr = 0,                   			   							\
	.read_ptr = 0,         		  											\
	.elements	= 0,	    			   									\
	.length = QUEUE_LENGTH,								                  	\
	.max_elements = 0,					                  					\
	.buffer = (uint8_t *)cyclic_name##_buff,							    \
	.overflow_allowed = OVERFLOW_ALLOWED,					          		\
};

void cyclic_byte_clear(CyclicByteBuffer *cyclic);

void cyclic_byte_add(CyclicByteBuffer *cyclic, uint8_t data);

bool cyclic_byte_get(CyclicByteBuffer *cyclic, uint8_t *data);

uint32_t cyclic_byte_get_elements(CyclicByteBuffer *cyclic);

uint32_t cyclic_byte_get_max_elements(CyclicByteBuffer *cyclic);

#endif
