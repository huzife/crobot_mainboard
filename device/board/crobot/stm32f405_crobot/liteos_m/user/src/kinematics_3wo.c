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
const double DISTANCE = 0.172;

void kinematics_inverse() {
    // corrected linear and angular
    double linear_x = k_inverse.velocity.linear_x / linear_factor;
    double linear_y = k_inverse.velocity.linear_y / linear_factor;
    double angular = k_inverse.velocity.angular_z / angular_factor;

    // rotate speed of each wheel, rad/s
    double v = angular * DISTANCE;
    double speed_left = v - linear_x * M_SQRT3 / 2 + linear_y / 2;
    double speed_back = v - linear_y;
    double speed_right = v + linear_x * M_SQRT3 / 2 + linear_y / 2;

    // motor speed(count in an interval time)
    double factor = pid_interval * CPR / (2 * M_PI);
    k_inverse.speed[0] = speed_left * factor;
    k_inverse.speed[1] = speed_back * factor;
    k_inverse.speed[2] = speed_right * factor;
}

void kinematics_forward() {
    // rotate speed
    double factor = pid_interval * CPR / (2 * M_PI);
    double speed_left = k_forward.speed[0] / factor;
    double speed_back = k_forward.speed[1] / factor;
    double speed_right = k_forward.speed[2] / factor;

    // linear and angular
    double linear_x = (speed_right - speed_left) / M_SQRT3;
    double linear_y = (speed_left - 2 * speed_back + speed_right) / 3;
    double angular = (speed_left + speed_back + speed_right) / 3;

    // correct
    k_forward.velocity.linear_x = linear_x * linear_factor;
    k_forward.velocity.linear_y = linear_y * linear_factor;
    k_forward.velocity.angular_z = angular * angular_factor;
}
