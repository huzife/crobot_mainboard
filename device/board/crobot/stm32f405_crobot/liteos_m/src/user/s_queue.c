#include "s_queue.h"
#include "los_atomic.h"
#include "los_memory.h"

inline uint32_t next_index(SQueue* queue, uint32_t cur) {
    return (cur + 1) % queue->size;
}

void squeue_init(SQueue* queue, uint8_t* pool, uint32_t size) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = size + 1;
    queue->buf = (uint8_t*)LOS_MemAlloc(pool, queue->size);
}

uint8_t s_push(SQueue* queue, const uint8_t data) {
    uint32_t cur_tail = LOS_AtomicRead((const Atomic*)&queue->tail);
    uint32_t next_tail = next_index(queue, cur_tail);
    if (next_tail != LOS_AtomicRead((const Atomic*)&queue->head)) {
        queue->buf[cur_tail] = data;
        LOS_AtomicSet((Atomic*)&queue->tail, next_tail);
        return 1;
    }

    return 0;
}

uint8_t s_pop(SQueue* queue, uint8_t* data) {
    uint32_t cur_head = LOS_AtomicRead((const Atomic*)&queue->head);
    if (cur_head != LOS_AtomicRead((const Atomic*)&queue->tail)) {
        *data = queue->buf[cur_head];
        LOS_AtomicSet((Atomic*)&queue->head, next_index(queue, cur_head));
        return 1;
    }

    return 0;
}
