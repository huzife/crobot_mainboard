#include "swsr_queue.h"
#include "FreeRTOS.h"

inline uint32_t next_index(SWSR_Queue* queue, uint32_t cur) {
    return (cur + 1) % queue->size;
}

void swsr_queue_init(SWSR_Queue* queue, uint32_t size) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = size + 1;
    queue->buf = pvPortMalloc(queue->size);
}

bool swsr_queue_push(SWSR_Queue* queue, const uint8_t data) {
    uint32_t cur_tail = queue->tail;
    uint32_t next_tail = next_index(queue, cur_tail);
    if (next_tail != queue->head) {
        queue->buf[cur_tail] = data;
        queue->tail = next_tail;
        return true;
    }

    return false;
}

bool swsr_queue_pop(SWSR_Queue* queue, uint8_t* data) {
    uint32_t cur_head = queue->head;
    if (cur_head != queue->tail) {
        *data = queue->buf[cur_head];
        queue->head = next_index(queue, cur_head);
        return true;
    }

    return false;
}
