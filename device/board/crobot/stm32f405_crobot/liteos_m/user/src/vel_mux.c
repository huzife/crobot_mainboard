#include "vel_mux.h"
#include "kinematics.h"
#include "los_atomic.h"
#include "los_memory.h"
#include "los_mux.h"
#include "los_swtmr.h"
#include <stdbool.h>

#define VACANT -1

volatile int velocity_avaliable;

// velocity mux control block
static struct {
    uint32_t count;         // Registered source count
    uint32_t max_count;     // Max registered source count
    uint8_t* priority;      // Priority map
    uint16_t* expiry_time;  // Expiry time map
    uint32_t* expiry_timer; // Expiry timer map
    uint32_t set_vel_mtx;   // Mutex for setting velocity
    int active_id;          // Active velocity source id
} vel_mux_cb;

static void vel_mux_timer_callback(uint32_t id) {
    if (vel_mux_cb.active_id == id)
        vel_mux_cb.active_id = VACANT;
}

void vel_mux_init(uint8_t* mem_pool, uint32_t max_vel_source_count) {
    velocity_avaliable = 0;
    vel_mux_cb.count = 0;
    vel_mux_cb.max_count = max_vel_source_count;
    vel_mux_cb.priority = (uint8_t*)LOS_MemAlloc(
                              mem_pool, max_vel_source_count);
    vel_mux_cb.expiry_time = (uint16_t*)LOS_MemAlloc(
                                 mem_pool, max_vel_source_count * 2);
    vel_mux_cb.expiry_timer = (uint32_t*)LOS_MemAlloc(
                                  mem_pool, max_vel_source_count * 4);
    vel_mux_cb.active_id = VACANT;
    LOS_MuxCreate(&vel_mux_cb.set_vel_mtx);
}

int vel_mux_register(uint32_t priority, uint16_t expiry_time) {
    if (vel_mux_cb.count >= vel_mux_cb.max_count)
        return -1;

    int index = LOS_AtomicIncRet((int*)&vel_mux_cb.count) - 1;
    vel_mux_cb.priority[index] = priority;
    vel_mux_cb.expiry_time[index] = expiry_time > MAX_EXPIRY_TIME
                                  ? MAX_EXPIRY_TIME
                                  : expiry_time;
    LOS_SwtmrCreate(vel_mux_cb.expiry_time[index],
                    LOS_SWTMR_MODE_NO_SELFDELETE,
                    vel_mux_timer_callback,
                    &vel_mux_cb.expiry_timer[index],
                    index);
    return index;
}

void vel_mux_set_velocity(Velocity_Message* velocity) {
    uint32_t id = velocity->id;
    if (id >= vel_mux_cb.count)
        return;

    LOS_SwtmrStop(vel_mux_cb.expiry_timer[id]);
    LOS_SwtmrStart(vel_mux_cb.expiry_timer[id]);

    // check active id
    LOS_MuxPend(vel_mux_cb.set_vel_mtx, 100);
    int active_id = vel_mux_cb.active_id;
    if (active_id == VACANT || active_id == id ||
        vel_mux_cb.priority[id] < vel_mux_cb.priority[active_id]) {
        // update current priority
        if (active_id != id)
            vel_mux_cb.active_id = id;

        // set velocity
        kinematics[0].linear_x = velocity->linear_x;
        kinematics[0].angular_z = velocity->angular_z;
        LOS_AtomicSet(&velocity_avaliable, 1);
    }
    LOS_MuxPost(vel_mux_cb.set_vel_mtx);
}
