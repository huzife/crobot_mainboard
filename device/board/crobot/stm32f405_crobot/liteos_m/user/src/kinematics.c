#include "kinematics.h"
#include "los_mux.h"
#include "los_tick.h"
#include <math.h>
#include <stdbool.h>

uint32_t kinematics_task_id;
Kinematics k_inverse;
Kinematics k_forward;
Odometry odometry;
uint32_t target_velocity_mtx;
uint32_t current_velocity_mtx;
static uint64_t last_tick = 0;

void kinematics_init() {
    LOS_MuxCreate(&target_velocity_mtx);
    LOS_MuxCreate(&current_velocity_mtx);
}

void kinematics_update_odom() {
    // get interval, reset counter
    uint64_t cur_tick = LOS_TickCountGet();
    float interval = (cur_tick - last_tick) / 1000.0;
    last_tick = cur_tick;

    // update odom
    LOS_MuxPend(current_velocity_mtx, 50);
    float vx = k_forward.velocity.linear_x;
    float vy = k_forward.velocity.linear_y;
    float angular = k_forward.velocity.angular_z;
    LOS_MuxPost(current_velocity_mtx);

    float dir = odometry.direction;
    float delta_x = (vx * cosf(dir) - vy * sinf(dir)) * interval;
    float delta_y = (vx * sinf(dir) + vy * cosf(dir)) * interval;
    float delta_yaw = angular * interval;

    odometry.position_x += delta_x;
    odometry.position_y += delta_y;
    odometry.direction += delta_yaw;
}

Odometry kinematics_get_odom() {
    return odometry;
}

void kinematics_set_target_velocity(Velocity velocity) {
    LOS_MuxPend(target_velocity_mtx, 50);
    k_inverse.velocity = velocity;
    LOS_MuxPost(target_velocity_mtx);
}

Velocity kinematics_get_current_velocity() {
    LOS_MuxPend(current_velocity_mtx, 50);
    Velocity velocity = k_forward.velocity;
    LOS_MuxPost(current_velocity_mtx);
    return velocity;
}

void kinematics_set_current_motor_speed(int16_t* speeds) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        k_forward.speed[i] = speeds[i];
    }
}

int16_t* kinematics_get_target_motor_speed() {
    return k_inverse.speed;
}
