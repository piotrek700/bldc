#ifndef CYCLIC_PTR_H_
#define CYCLIC_PTR_H_

#include <stdint.h>
#include <stdbool.h>

typedef void * type_buff;

typedef struct {
	uint32_t write_ptr;
	uint32_t read_ptr;
	uint32_t elements;
	uint32_t length;
	uint32_t max_elements;
	type_buff *buffer;
	bool overflow_allowed;
} CyclicPtrBuffer;

#define CYCLIC_BUFFER_PTR_DEF(_cyclic_name, _overflow_allowed, _queue_length)	\
static volatile type_buff _cyclic_name##_buff[_queue_length];					\
static volatile CyclicPtrBuffer _cyclic_name ={              					\
	.write_ptr = 0,                   			   								\
	.read_ptr = 0,         		  												\
	.elements	= 0,	    			   										\
	.length = _queue_length,								                  	\
	.max_elements = 0,					                  						\
	.buffer = (type_buff *)_cyclic_name##_buff,							    	\
	.overflow_allowed = _overflow_allowed,					          			\
};

void cyclic_ptr_clear(CyclicPtrBuffer *cyclic);

void cyclic_ptr_add(CyclicPtrBuffer *cyclic, type_buff data);

bool cyclic_ptr_get(CyclicPtrBuffer *cyclic, type_buff *data);

uint32_t cyclic_ptr_get_elements(CyclicPtrBuffer *cyclic);

uint32_t cyclic_ptr_get_max_elements(CyclicPtrBuffer *cyclic);

#endif
