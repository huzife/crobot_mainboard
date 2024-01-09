#include "vel_mux.h"
#include "kinematics.h"
#include "modbus_rtu.h"
#include "los_memory.h"
#include "los_task.h"
#include "tim.h"
#include <stdbool.h>

static struct {
    uint32_t count;
    uint32_t max_count;
    uint8_t* priority;
    uint16_t* expiry_time;
    uint8_t current_priority;
} vel_mux_info;

static Kinematics_2WD kinematics;

void vel_mux_init(uint8_t* mem_pool, uint32_t max_vel_source_count) {
    vel_mux_info.count = 0;
    vel_mux_info.max_count = max_vel_source_count;
    vel_mux_info.priority = (uint8_t*)LOS_MemAlloc(mem_pool, max_vel_source_count);
    vel_mux_info.expiry_time = (uint16_t*)LOS_MemAlloc(mem_pool, max_vel_source_count * 2);
    vel_mux_info.current_priority = 0xFF;
    kinematics_2WD_init(&kinematics);
}

int vel_mux_register(uint32_t priority, uint16_t expiry_time) {
    if (vel_mux_info.count >= vel_mux_info.max_count)
        return -1;

    int index = vel_mux_info.count++;
    vel_mux_info.priority[index] = priority;
    vel_mux_info.expiry_time[index] = expiry_time;
    return index;
}

void vel_mux_set_velocity(Velocity_Message* velocity) {
    uint32_t id = velocity->id;
    if (id >= vel_mux_info.count)
        return;

    if (vel_mux_info.priority[id] <= vel_mux_info.current_priority) {
        uint16_t time = vel_mux_info.expiry_time[id];
        if (time && time <= 2000) {
            vel_mux_info.current_priority = vel_mux_info.priority[id];
            htim7.Instance->ARR = time * 10;
            htim7.Instance->CNT = 0;
            if (htim7.State == HAL_TIM_STATE_READY)
                HAL_TIM_Base_Start_IT(&htim7);
        }

        kinematics.linear_x = velocity->linear_x;
        kinematics.angular_z = velocity->angular_z;
        kinematics_2WD_inverse(&kinematics);
    }
    modbus_rtu_set_holding_regs(0, 1, 0, (uint16_t*)kinematics.speed, 2);
}

void vel_mux_expire_callback() {
    vel_mux_info.current_priority = 0xFF;
    HAL_TIM_Base_Stop_IT(&htim7);
}
