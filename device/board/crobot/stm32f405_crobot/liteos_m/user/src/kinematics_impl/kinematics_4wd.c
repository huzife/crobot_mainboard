#ifdef LOSCFG_ROBOT_BASE_4WD
#include "kinematics_impl.h"
#include <math.h>
#include <stdbool.h>

const float RADIUS = 0.0325f;
const float SEPARATION = 0.172f;

void kinematics_update_odometry(Odometry* odometry, Velocity velocity, float dt) {
    float dx = velocity.linear_x * dt;
    float dyaw = velocity.angular_z * dt;

    if (fabsf(dyaw) < 1e-6f) {
        float dir = odometry->direction + dyaw * 0.5f;
        odometry->position_x += dx * cosf(dir);
        odometry->position_y += dx * sinf(dir);
        odometry->direction += dyaw;
    } else {
        float dir_old = odometry->direction;
        float r = dx / dyaw;
        odometry->direction += dyaw;
        odometry->position_x += r * (sinf(odometry->direction) - sinf(dir_old));
        odometry->position_y += -r * (cosf(odometry->direction) - cosf(dir_old));
    }
}

void kinematics_inverse_func(Velocity velocity, float speeds[]) {
    float linear = velocity.linear_x;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    float speed_left = (angular * SEPARATION / 2 - linear) / RADIUS;
    float speed_right = (angular * SEPARATION / 2 + linear) / RADIUS;

    speeds[0] = speed_left;
    speeds[1] = speed_right;
    speeds[2] = speed_left;
    speeds[3] = speed_right;
}

void kinematics_forward_func(float speeds[], Velocity* velocity) {
    // rotate speed
    float speed_left = (speeds[0] + speeds[2]) / 2;
    float speed_right = (speeds[1] + speeds[3]) / 2;

    // linear and angular
    velocity->linear_x = (RADIUS * (speed_right - speed_left)) / 2;
    velocity->linear_y = 0.0f;
    velocity->angular_z = (RADIUS * (speed_right + speed_left)) / SEPARATION;
}
#endif
