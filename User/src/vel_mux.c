#include "vel_mux.h"
#include "crobot_atomic.h"
#include "kinematics.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include <stdbool.h>

#define VACANT -1

// velocity mux control block
static struct {
    uint32_t count;                 // Registered source count
    uint32_t max_count;             // Max registered source count
    uint8_t* priority;              // Priority map
    uint16_t* expiry_time;          // Expiry time map
    TimerHandle_t* expiry_timer;    // Expiry timer map
    SemaphoreHandle_t set_vel_mtx;  // Mutex for setting velocity
    int active_id;                  // Active velocity source id
} vel_mux_cb;

static void vel_mux_timer_callback(TimerHandle_t timer) {
    uint32_t id = (uint32_t)pvTimerGetTimerID(timer);
    if (vel_mux_cb.active_id == id)
        vel_mux_cb.active_id = VACANT;
}

void vel_mux_init(uint32_t max_vel_source_count) {
    vel_mux_cb.count = 0;
    vel_mux_cb.max_count = max_vel_source_count;
    vel_mux_cb.priority = pvPortMalloc(max_vel_source_count);
    vel_mux_cb.expiry_time = pvPortMalloc(max_vel_source_count * 2);
    vel_mux_cb.expiry_timer = pvPortMalloc(max_vel_source_count * sizeof(TimerHandle_t));
    vel_mux_cb.active_id = VACANT;
    vel_mux_cb.set_vel_mtx = xSemaphoreCreateMutex();
}

int vel_mux_register(uint32_t priority, uint16_t expiry_time) {
    if (vel_mux_cb.count >= vel_mux_cb.max_count)
        return -1;

    int index = atomic_add((volatile int*)&vel_mux_cb.count, 1) - 1;
    vel_mux_cb.priority[index] = priority;
    vel_mux_cb.expiry_time[index] = expiry_time > MAX_EXPIRY_TIME
                                  ? MAX_EXPIRY_TIME
                                  : expiry_time;
    vel_mux_cb.expiry_timer[index] =
        xTimerCreate(NULL, vel_mux_cb.expiry_time[index],
                     pdFALSE, (void*)index, vel_mux_timer_callback);
    return index;
}

void vel_mux_set_velocity(Velocity_Message velocity) {
    uint32_t id = velocity.id;
    if (id >= vel_mux_cb.count)
        return;

    xTimerReset(vel_mux_cb.expiry_timer[id], portMAX_DELAY);

    // check active id
    xSemaphoreTake(vel_mux_cb.set_vel_mtx, 100);
    int active_id = vel_mux_cb.active_id;
    if (active_id == VACANT || active_id == id ||
        vel_mux_cb.priority[id] < vel_mux_cb.priority[active_id]) {
        // update current priority
        if (active_id != id)
            vel_mux_cb.active_id = id;

        // set velocity
        kinematics_set_target_velocity(velocity.velocity);
    }
    xSemaphoreGive(vel_mux_cb.set_vel_mtx);
}
