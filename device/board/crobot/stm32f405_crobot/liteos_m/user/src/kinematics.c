#include "kinematics.h"

const double PI = 3.1415926535;
const double RADIUS = 0.0325;
const double SEPARATION = 0.172;
const double INTERVAL = 0.05;
const uint16_t COUNT_PER_CYCLE = 1580;
const bool REVERSE[2] = {false, true};

volatile Kinematics_2WD_Param kinematics_param = {
    RADIUS,
    SEPARATION,
    INTERVAL,
    COUNT_PER_CYCLE,
    {false, true}
};

void kinematics_2WD_init(Kinematics_2WD* kinematics) {
    kinematics->linear_x = 0;
    kinematics->angular_z = 0;
    kinematics->speed[0] = 0;
    kinematics->speed[1] = 0;
}

void kinematics_2WD_inverse(Kinematics_2WD* kinematics) {
    double radius = kinematics_param.radius;
    double separation = kinematics_param.separation;
    double interval = kinematics_param.interval;
    uint16_t count_per_cycle = kinematics_param.count_per_cycle;
    double linear = kinematics->linear_x;
    double angular = kinematics->angular_z;

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

    kinematics->linear_x = (radius * (speed_left + speed_right)) / 2;
    kinematics->angular_z = (radius * (speed_right - speed_left)) / separation;
}
