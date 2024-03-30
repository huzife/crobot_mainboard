#ifndef USER_BUMPER_H
#define USER_BUMPER_H

#include <stdbool.h>
#include <stdint.h>

extern uint32_t bumper_task_id;

typedef struct {
    bool left;  // true: hit left
    bool front; // true: hit front
    bool right; // true: hit right
} Bumper_State;

/// @brief Init bumper
void bumper_init();

/// @brief Check bumper state
/// @return Bumper_State
Bumper_State bumper_check();

#endif // USER_BUMPER_H
