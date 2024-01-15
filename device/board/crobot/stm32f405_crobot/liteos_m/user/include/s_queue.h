#ifndef USER_S_QUEUE_H
#define USER_S_QUEUE_H

#include <stdint.h>

typedef struct {
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t size;
    uint8_t* buf;
} SQueue;

void squeue_init(SQueue* queue, uint8_t* pool, uint32_t size);
uint8_t s_push(SQueue* queue, const uint8_t data);
uint8_t s_pop(SQueue* queue, uint8_t* data);

#endif // USER_S_QUEUE_H
