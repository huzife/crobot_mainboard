#include "kinematics_impl/kinematics_3wo.h"
#include <math.h>

const float SQRT3 = M_SQRT3;

static Kinematics_3WO_Param kinematics_param;

void kinematics_set_param_3wo(Kinematics_3WO_Param param) {
    kinematics_param = param;
}

void kinematics_update_odometry_3wo(Odometry* odometry, Velocity velocity, float dt) {
    float dx = velocity.linear_x * dt;
    float dy = velocity.linear_y * dt;
    float dyaw = velocity.angular_z * dt;

    float dir = odometry->direction;
    odometry->position_x += dx * cosf(dir) - dy * sinf(dir);
    odometry->position_y += dx * sinf(dir) + dy * cosf(dir);
    odometry->direction += dyaw;
}

void kinematics_inverse_3wo(Velocity velocity, float speeds[]) {
    float radius = kinematics_param.radius;
    float distance = kinematics_param.distance;
    float linear_x = velocity.linear_x;
    float linear_y = velocity.linear_y;
    float angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    float v = angular * distance;
    speeds[0] = (v - linear_x * SQRT3 / 2 + linear_y / 2) / radius;
    speeds[1] = (v - linear_y) / radius;
    speeds[2] = (v + linear_x * SQRT3 / 2 + linear_y / 2) / radius;
}

void kinematics_forward_3wo(float speeds[], Velocity* velocity) {
    float radius = kinematics_param.radius;
    float distance = kinematics_param.distance;

    // rotate speed
    float speed_left = speeds[0];
    float speed_back = speeds[1];
    float speed_right = speeds[2];

    // linear and angular
    velocity->linear_x = (speed_right - speed_left) * radius / SQRT3;
    velocity->linear_y = (speed_left - 2 * speed_back + speed_right) * radius / 3;
    velocity->angular_z = (speed_left + speed_back + speed_right) * radius / 3 / distance;
}
