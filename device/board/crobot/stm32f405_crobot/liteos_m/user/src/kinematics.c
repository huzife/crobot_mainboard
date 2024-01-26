#include "kinematics.h"
#include <math.h>

const double PI = 3.1415926535;
const double RADIUS = 0.0325;
const double SEPARATION = 0.172;
const double INTERVAL = 0.05;
const uint16_t COUNT_PER_CYCLE = 1580;
const bool REVERSE[2] = {false, true};

Kinematics_2WD_Param kinematics_param = {
    RADIUS,
    SEPARATION,
    1.0,
    1.0,
    INTERVAL,
    COUNT_PER_CYCLE,
    {false, true}
};

Kinematics_2WD kinematics[2];
Odometry odom;

void kinematics_2WD_init(Kinematics_2WD* kinematics) {
    kinematics->linear_x = 0;
    kinematics->angular_z = 0;
    kinematics->speed[0] = 0;
    kinematics->speed[1] = 0;
}

void odom_init(Odometry* odometry) {
    odometry->position_x = 0.0;
    odometry->position_y = 0.0;
    odometry->direction = 0.0;
}

void update_odom(Odometry* odometry, float linear, float angular,
                 float interval) {
    float dir = odometry->direction;
    float delta_x = linear * cosf(dir) * interval;
    float delta_y = linear * sinf(dir) * interval;
    float delta_yaw = angular * interval;

    odometry->position_x += delta_x;
    odometry->position_y += delta_y;
    odometry->direction += delta_yaw;
}

void kinematics_2WD_inverse(Kinematics_2WD* kinematics) {
    double radius = kinematics_param.radius;
    double separation = kinematics_param.separation;
    double interval = kinematics_param.interval;
    uint16_t count_per_cycle = kinematics_param.count_per_cycle;
    double linear = kinematics->linear_x / kinematics_param.linear_factor;
    double angular = kinematics->angular_z / kinematics_param.angular_factor;

    kinematics->speed[1] = (linear - angular * separation / 2) / radius *
                               interval * count_per_cycle / (2 * PI);
    kinematics->speed[0] = (linear + angular * separation / 2) / radius *
                               interval * count_per_cycle / (2 * PI);
    if (kinematics_param.reverse[0])
        kinematics->speed[0] = -kinematics->speed[0];
    if (kinematics_param.reverse[1])
        kinematics->speed[1] = -kinematics->speed[1];
}

void kinematics_2WD_forward(Kinematics_2WD* kinematics) {
    double radius = kinematics_param.radius;
    double separation = kinematics_param.separation;
    double interval = kinematics_param.interval;
    uint16_t count_per_cycle = kinematics_param.count_per_cycle;

    double speed_right = (double)kinematics->speed[0] * 2 * PI /
                            (count_per_cycle * interval);
    double speed_left = (double)kinematics->speed[1] * 2 * PI /
                            (count_per_cycle * interval);
    if (kinematics_param.reverse[0])
        speed_right = -speed_right;
    if (kinematics_param.reverse[1])
        speed_left = -speed_left;

    kinematics->linear_x = (radius * (speed_left + speed_right)) / 2 *
                           kinematics_param.linear_factor;
    kinematics->angular_z = (radius * (speed_right - speed_left)) / separation *
                            kinematics_param.angular_factor;
}
