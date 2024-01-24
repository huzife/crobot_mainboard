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

/// @brief Init SWSR_Queue
/// @param[out] queue Pointer to SWSR_Queue
/// @param[in] pool Address of memory pool
/// @param[in] size Queue size
void swsr_queue_init(SWSR_Queue* queue, uint8_t* pool, uint32_t size);

/// @brief Push a byte into the end of queue
/// @param[in|out] Queue Pointer to SWSR_Queue
/// @param[in] data Byte to push
/// @return bool
/// @retval true Succeed pushing
/// @retval false Failed to push data, maybe the queue is full
bool swsr_queue_push(SWSR_Queue* queue, const uint8_t data);

/// @brief Pop a byte from the head of queue
/// @param[in|out] Queue Pointer to SWSR_Queue
/// @param[in] data Pointer to received data
/// @return bool
/// @retval true Succeed popping a byte
/// @retval false Failed to pop data, maybe the queue is empty
bool swsr_queue_pop(SWSR_Queue* queue, uint8_t* data);

#endif // USER_SWSR_QUEUE_H
