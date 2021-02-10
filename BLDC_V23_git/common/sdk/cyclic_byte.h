#ifndef CYCLIC_BYTE_H_
#define CYCLIC_BYTE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t length;
	uint32_t max_elements;
	uint8_t *buffer;
	bool overflow_allowed;
} CyclicByteBuffer_t;

#define CYCLIC_BUFFER_BYTE_DEF(_cyclic_name, _overflow_allowed, _queue_length)	\
static volatile uint8_t _cyclic_name##_buff[_queue_length];						\
static volatile CyclicByteBuffer_t _cyclic_name ={              				\
	.write_ptr = 0,                   			   								\
	.read_ptr = 0,         		  												\
	.elements	= 0,	    			   										\
	.length = _queue_length,								                	\
	.max_elements = 0,					                  						\
	.buffer = (uint8_t *)_cyclic_name##_buff,							    	\
	.overflow_allowed = _overflow_allowed,					          			\
};

void cyclic_byte_clear(CyclicByteBuffer_t *cyclic);

void cyclic_byte_add(CyclicByteBuffer_t *cyclic, uint8_t data);

bool cyclic_byte_get(CyclicByteBuffer_t *cyclic, uint8_t *data);

uint32_t cyclic_byte_get_elements(CyclicByteBuffer_t *cyclic);

uint32_t cyclic_byte_get_max_elements(CyclicByteBuffer_t *cyclic);

#endif
