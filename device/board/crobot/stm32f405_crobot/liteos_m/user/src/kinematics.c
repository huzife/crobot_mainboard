#include "kinematics.h"
#include <math.h>
#include <stdbool.h>

Kinematics k_inverse = {0};
Kinematics k_forward = {0};
Odometry odometry = {0};

void kinematics_update_odom(float interval) {
    float vx = k_forward.velocity.linear_x;
    float vy = k_forward.velocity.linear_y;
    float angular = k_forward.velocity.angular_z;

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
    k_inverse.velocity = velocity;
}

Velocity kinematics_get_current_velocity() {
    return k_forward.velocity;
}

void kinematics_set_current_motor_speed(uint16_t* speeds) {
    for (int i = 0; i < MOTOR_NUM; i++) {
        k_forward.speed[i] = speeds[i];
    }
}

uint16_t* kinematics_get_target_motor_speed() {
    return k_inverse.speed;
}
