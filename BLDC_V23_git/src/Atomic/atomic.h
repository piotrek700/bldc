#ifndef ATOMIC_H_
#define ATOMIC_H_

#include "platform.h"

void enter_critical(void);

void exit_critical(void);

uint32_t critiacl_get_max_queue_depth(void);

void safe_increment(uint32_t *addr);

void safe_decrement(uint32_t *addr);

#endif
