#ifdef LOSCFG_ROBOT_BASE_3WO
#include "kinematics_impl.h"
#include <math.h>
#include <stdbool.h>

const float SQRT3 = M_SQRT3;
const float RADIUS = 0.0325f;
const float DISTANCE = 0.172f;

void kinematics_update_odometry(Odometry* odometry, Velocity velocity, float dt) {
    float dx = velocity.linear_x * dt;
    float dy = velocity.linear_y * dt;
    float dyaw = velocity.angular_z * dt;

    float dir = odometry->direction;
    odometry->position_x += dx * cosf(dir) - dy * sinf(dir);
    odometry->position_y += dx * sinf(dir) + dy * cosf(dir);
    odometry->direction += dyaw;
}

void kinematics_inverse_func(Velocity velocity, float speeds[]) {
    float linear_x = velocity.linear_x;
    float linear_y = velocity.linear_y;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    float v = angular * DISTANCE;
    speeds[0] = v - linear_x * SQRT3 / 2 + linear_y / 2;
    speeds[1] = v - linear_y;
    speeds[2] = v + linear_x * SQRT3 / 2 + linear_y / 2;
}

void kinematics_forward_func(float speeds[], Velocity* velocity) {
    // rotate speed
    float speed_left = speeds[0];
    float speed_back = speeds[1];
    float speed_right = speeds[2];

    // linear and angular
    velocity->linear_x = (speed_right - speed_left) / SQRT3;
    velocity->linear_y = (speed_left - 2 * speed_back + speed_right) / 3;
    velocity->angular_z = (speed_left + speed_back + speed_right) / 3;
}
#endif
