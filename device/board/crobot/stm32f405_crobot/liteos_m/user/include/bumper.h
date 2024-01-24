#ifndef USER_BUMPER_H
#define USER_BUMPER_H

#include <stdbool.h>

typedef struct {
    bool left;
    bool front;
    bool right;
} Bumper_State;

Bumper_State bumper_check();

#endif // USER_BUMPER_H
