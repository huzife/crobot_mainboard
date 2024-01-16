#ifndef USER_SWSR_QUEUE_H
#define USER_SWSR_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t size;
    uint8_t* buf;
} SWSR_Queue;

void swsr_queue_init(SWSR_Queue* queue, uint8_t* pool, uint32_t size);
bool swsr_queue_push(SWSR_Queue* queue, const uint8_t data);
bool swsr_queue_pop(SWSR_Queue* queue, uint8_t* data);

#endif // USER_SWSR_QUEUE_H
