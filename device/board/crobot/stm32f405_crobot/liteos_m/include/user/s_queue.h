#ifndef S_QUEUE_H
#define S_QUEUE_H

#include <stdint.h>

typedef struct {
    uint32_t head;
    uint32_t tail;
    uint8_t buf[128];
    uint32_t size;
} SQueue;

void squeue_init(SQueue* queue);
uint8_t s_push(SQueue* queue, const uint8_t data);
uint8_t s_pop(SQueue* queue, uint8_t* data);

#endif // S_QUEUE_H
