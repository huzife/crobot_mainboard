#ifndef USER_VEL_MUX_H
#define USER_VEL_MUX_H

#include "los_event.h"
#include "los_queue.h"

extern EVENT_CB_S vel_mux_event;

typedef struct {
    uint32_t id;
    double linear_x;
    double angular_z;
} Velocity_Message;

void vel_mux_init(uint8_t* mem_pool, uint32_t max_vel_source_count);
int vel_mux_register(uint32_t priority, uint16_t expiry_time);
void vel_mux_set_velocity(Velocity_Message* velocity);

#endif // USER_VEL_MUX_H
