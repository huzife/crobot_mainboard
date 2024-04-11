#ifdef LOSCFG_ROBOT_BASE_2WD
#include "kinematics_impl.h"
#include <math.h>
#include <stdbool.h>

const double RADIUS = 0.0325;
const double SEPARATION = 0.172;

void kinematics_update_odometry(Odometry* odometry, Velocity velocity, double dt) {
    double dx = velocity.linear_x * dt;
    double dyaw = velocity.angular_z * dt;

    if (fabs(dyaw) < 1e-6) {
        double dir = odometry->direction + dyaw * 0.5;
        odometry->position_x += dx * cos(dir);
        odometry->position_y += dx * sin(dir);
        odometry->direction += dyaw;
    } else {
        double dir_old = odometry->direction;
        double r = dx / dyaw;
        odometry->direction += dyaw;
        odometry->position_x += r * (sin(odometry->direction) - sin(dir_old));
        odometry->position_y += -r * (cos(odometry->direction) - cos(dir_old));
    }
}

void kinematics_inverse_func(Velocity velocity, double speeds[]) {
    double linear = velocity.linear_x;
    double angular = velocity.angular_z;

    // rotate speed of each wheel, rad/s
    speeds[0] = (angular * SEPARATION / 2 - linear) / RADIUS;
    speeds[1] = (angular * SEPARATION / 2 + linear) / RADIUS;
}

void kinematics_forward_func(double speeds[], Velocity* velocity) {
    // rotate speed
    double speed_left = speeds[0];
    double speed_right = speeds[1];

    // linear and angular
    velocity->linear_x = (RADIUS * (speed_right - speed_left)) / 2;
    velocity->linear_y = 0.0;
    velocity->angular_z = (RADIUS * (speed_right + speed_left)) / SEPARATION;
}
#endif
