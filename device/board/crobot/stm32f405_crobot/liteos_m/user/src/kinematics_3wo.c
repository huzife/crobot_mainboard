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
    double speed[WHEEL_NUM];
    double v = angular * DISTANCE;
    speed[0] = v - linear_x * M_SQRT3 / 2 + linear_y / 2;
    speed[1] = v - linear_y;
    speed[2] = v + linear_x * M_SQRT3 / 2 + linear_y / 2;

    // motor speed(count in an interval time)
    double factor = pid_interval * CPR / (2 * M_PI);
    for (int i = 0; i < WHEEL_NUM; i++) {
        k_inverse.speed[i] = speed[i] * factor;
    }
}

void kinematics_forward() {
    // rotate speed
    double factor = pid_interval * CPR / (2 * M_PI);
    double speed[WHEEL_NUM] = {
        (double)k_forward.speed[0] / factor,
        (double)k_forward.speed[1] / factor,
        (double)k_forward.speed[2] / factor
    };

    // linear and angular
    double linear_x = (speed[2] - speed[0]) / M_SQRT3;
    double linear_y = (speed[0] - 2 * speed[1] + speed[2]) / 3;
    double angular = (speed[0] + speed[1] + speed[2]) / 3;

    // correct
    k_forward.velocity.linear_x = linear_x * linear_factor;
    k_forward.velocity.linear_y = linear_y * linear_factor;
    k_forward.velocity.angular_z = angular * angular_factor;
}
