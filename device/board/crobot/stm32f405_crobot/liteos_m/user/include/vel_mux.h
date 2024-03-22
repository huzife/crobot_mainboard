#ifndef USER_VEL_MUX_H
#define USER_VEL_MUX_H

#include "velocity.h"
#include <stdint.h>

#define MAX_EXPIRY_TIME 1000

typedef struct {
    uint32_t id;
    Velocity velocity;
} Velocity_Message;

extern volatile int velocity_avaliable;

/// @brief Init vel_mux module
/// @param[in] mem_pool Address of memory pool
/// @param[in] max_vel_source_count The maximum number of velocity source
void vel_mux_init(uint8_t* mem_pool, uint32_t max_vel_source_count);

/// @brief Register a velocity source to vel_mux
/// @param[in] priority Priority of this velocity source
/// @param[in] expiry_time How long this priority will last
/// @return int
/// @retval -1 Failed to register
/// @retval [0 ~ INT_MAX] Your id
int vel_mux_register(uint32_t priority, uint16_t expiry_time);

/// @brief Set velocity
/// @param[in] velocity Velocity message
void vel_mux_set_velocity(Velocity_Message velocity);

#endif // USER_VEL_MUX_H
