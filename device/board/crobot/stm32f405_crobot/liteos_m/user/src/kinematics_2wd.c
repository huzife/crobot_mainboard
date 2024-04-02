#include "kinematics.h"
#include <math.h>
#include <stdbool.h>

// parameters
extern Kinematics k_inverse;
extern Kinematics k_forward;
extern Odometry odometry;

double pid_interval = 0.05;
double linear_factor = 1.0;
double angular_factor = 1.0;
const double CPR = 1580;
const double RADIUS = 0.0325;
const double SEPARATION = 0.172;

void kinematics_inverse() {
    // corrected linear and angular
    double linear = k_inverse.velocity.linear_x / linear_factor;
    double angular = k_inverse.velocity.angular_z / angular_factor;

    // rotate speed of each wheel, rad/s
    double speed_left = (angular * SEPARATION / 2 - linear) / RADIUS;
    double speed_right = (angular * SEPARATION / 2 + linear) / RADIUS;

    // motor speed(count in an interval time)
    double factor = pid_interval * CPR / (2 * M_PI);
    k_inverse.speed[0] = speed_left * factor;
    k_inverse.speed[1] = speed_right * factor;
}

void kinematics_forward() {
    // rotate speed
    double factor = pid_interval * CPR / (2 * M_PI);
    double speed_left = (double)k_forward.speed[0] / factor;
    double speed_right = (double)k_forward.speed[1] / factor;

    // linear and angular
    double linear = (RADIUS * (speed_right - speed_left)) / 2;
    double angular = (RADIUS * (speed_right + speed_left)) / SEPARATION;

    // correct
    k_forward.velocity.linear_x = linear * linear_factor;
    k_forward.velocity.angular_z = angular * angular_factor;
}
