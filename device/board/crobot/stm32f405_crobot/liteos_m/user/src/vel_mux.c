#include "vel_mux.h"
#include "kinematics.h"
#include "modbus_rtu.h"
#include "los_memory.h"
#include "los_mux.h"
#include "tim.h"
#include <stdbool.h>

// velocity mux control block
static struct {
    uint32_t count;         // Registered source count
    uint32_t max_count;     // Max registered source count
    uint8_t* priority;      // Priority map
    uint16_t* expiry_time;  // Expiry time map
    uint8_t cur_priority;   // Current priority
    uint32_t set_vel_mtx;   // Mutex for setting velocity
} vel_mux_cb;

static Kinematics_2WD kinematics;

void vel_mux_init(uint8_t* mem_pool, uint32_t max_vel_source_count) {
    vel_mux_cb.count = 0;
    vel_mux_cb.max_count = max_vel_source_count;
    vel_mux_cb.priority = (uint8_t*)LOS_MemAlloc(
                              mem_pool, max_vel_source_count);
    vel_mux_cb.expiry_time = (uint16_t*)LOS_MemAlloc(
                                 mem_pool, max_vel_source_count * 2);
    vel_mux_cb.cur_priority = 0xFF;
    LOS_MuxCreate(&vel_mux_cb.set_vel_mtx);
    kinematics_2WD_init(&kinematics);
}

int vel_mux_register(uint32_t priority, uint16_t expiry_time) {
    if (vel_mux_cb.count >= vel_mux_cb.max_count)
        return -1;

    int index = vel_mux_cb.count++;
    vel_mux_cb.priority[index] = priority;
    vel_mux_cb.expiry_time[index] = expiry_time > MAX_EXPIRY_TIME
                                  ? MAX_EXPIRY_TIME
                                  : expiry_time;
    return index;
}

void vel_mux_set_velocity(Velocity_Message* velocity) {
    uint32_t id = velocity->id;
    if (id >= vel_mux_cb.count)
        return;

    // check priority
    LOS_MuxPend(vel_mux_cb.set_vel_mtx, 100);
    if (vel_mux_cb.priority[id] <= vel_mux_cb.cur_priority) {
        // update current priority
        vel_mux_cb.cur_priority = vel_mux_cb.priority[id];

        // restart timer
        HAL_TIM_Base_Stop_IT(&htim7);
        htim7.Instance->ARR = vel_mux_cb.expiry_time[id] * 10;
        htim7.Instance->CNT = 0;
        HAL_TIM_Base_Start_IT(&htim7);

        // set velocity
        kinematics.linear_x = velocity->linear_x;
        kinematics.angular_z = velocity->angular_z;
        kinematics_2WD_inverse(&kinematics);
        modbus_rtu_set_holding_regs(0, 1, 0, (uint16_t*)kinematics.speed, 2);
    }
    LOS_MuxPost(vel_mux_cb.set_vel_mtx);
}

void vel_mux_expire_callback() {
    vel_mux_cb.cur_priority = 0xFF;
    HAL_TIM_Base_Stop_IT(&htim7);
}
