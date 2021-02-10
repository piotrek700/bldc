#ifndef CYCLIC_PTR_H_
#define CYCLIC_PTR_H_

#include <stdint.h>
#include <stdbool.h>

typedef void * BuffType_t;

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t length;
	uint32_t max_elements;
	BuffType_t *buffer;
	bool overflow_allowed;
} CyclicPtrBuffer_t;

#define CYCLIC_BUFFER_PTR_DEF(_cyclic_name, _overflow_allowed, _queue_length)	\
static volatile BuffType_t _cyclic_name##_buff[_queue_length];					\
static volatile CyclicPtrBuffer_t _cyclic_name ={              					\
	.write_ptr = 0,                   			   								\
	.read_ptr = 0,         		  												\
	.elements	= 0,	    			   										\
	.length = _queue_length,								                  	\
	.max_elements = 0,					                  						\
	.buffer = (BuffType_t *)_cyclic_name##_buff,							    \
	.overflow_allowed = _overflow_allowed,					          			\
};

void cyclic_ptr_clear(CyclicPtrBuffer_t *cyclic);

void cyclic_ptr_add(CyclicPtrBuffer_t *cyclic, BuffType_t data);

bool cyclic_ptr_get(CyclicPtrBuffer_t *cyclic, BuffType_t *data);

uint32_t cyclic_ptr_get_elements(CyclicPtrBuffer_t *cyclic);

uint32_t cyclic_ptr_get_max_elements(CyclicPtrBuffer_t *cyclic);

#endif
