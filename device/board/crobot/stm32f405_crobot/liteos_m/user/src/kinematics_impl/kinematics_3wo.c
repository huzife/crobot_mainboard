#ifdef LOSCFG_ROBOT_BASE_3WO
#include "kinematics_impl.h"
#include <math.h>
#include <stdbool.h>

const double RADIUS = 0.0325;
const double DISTANCE = 0.172;

void kinematics_update_odometry(Odometry* odometry, Velocity velocity, double dt) {
    double dx = velocity.linear_x * dt;
    double dy = velocity.linear_y * dt;
    double dyaw = velocity.angular_z * dt;

    double dir = odometry->direction;
    odometry->position_x += dx * cos(dir) - dy * sin(dir);
    odometry->position_y += dx * sin(dir) + dy * cos(dir);
    odometry->direction += dyaw;
}

void kinematics_inverse_func(Velocity velocity, double speeds[]) {
    double linear_x = velocity.linear_x;
    double linear_y = velocity.linear_y;
    double angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    double v = angular * DISTANCE;
    speeds[0] = v - linear_x * M_SQRT3 / 2 + linear_y / 2;
    speeds[1] = v - linear_y;
    speeds[2] = v + linear_x * M_SQRT3 / 2 + linear_y / 2;
}

void kinematics_forward_func(double speeds[], Velocity* velocity) {
    // rotate speed
    double speed_left = speeds[0];
    double speed_back = speeds[1];
    double speed_right = speeds[2];

    // linear and angular
    velocity->linear_x = (speed_right - speed_left) / M_SQRT3;
    velocity->linear_y = (speed_left - 2 * speed_back + speed_right) / 3;
    velocity->angular_z = (speed_left + speed_back + speed_right) / 3;
}
#endif
