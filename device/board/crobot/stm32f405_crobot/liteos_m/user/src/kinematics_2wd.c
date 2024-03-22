#include "kinematics.h"
#include <math.h>
#include <stdbool.h>

const double RADIUS = 0.0325;
const double SEPARATION = 0.172;
const double INTERVAL = 0.05;
const uint16_t COUNT_PER_CYCLE = 1580;
const bool REVERSE[MOTOR_NUM] = {false, true};

typedef struct {
    float radius;
    float separation;
    float linear_factor;
    float angular_factor;
    float interval;
    uint16_t count_per_cycle;
    bool reverse[MOTOR_NUM]; // 0: right, 1: left
} Kinematics_2WD_Param;

static Kinematics_2WD_Param kinematics_param = {
    RADIUS,
    SEPARATION,
    1.0,
    1.0,
    INTERVAL,
    COUNT_PER_CYCLE,
    {false, true}
};

extern Kinematics k_inverse;
extern Kinematics k_forward;
extern Odometry odometry;

void kinematics_inverse() {
    double radius = kinematics_param.radius;
    double separation = kinematics_param.separation;
    double interval = kinematics_param.interval;
    uint16_t count_per_cycle = kinematics_param.count_per_cycle;
    double linear = k_inverse.velocity.linear_x / kinematics_param.linear_factor;
    double angular = k_inverse.velocity.angular_z / kinematics_param.angular_factor;

    k_inverse.speed[1] = (linear - angular * separation / 2) / radius *
                             interval * count_per_cycle / (2 * M_PI);
    k_inverse.speed[0] = (linear + angular * separation / 2) / radius *
                             interval * count_per_cycle / (2 * M_PI);
    if (kinematics_param.reverse[0])
        k_inverse.speed[0] = -k_inverse.speed[0];
    if (kinematics_param.reverse[1])
        k_inverse.speed[1] = -k_inverse.speed[1];
}

void kinematics_forward() {
    double radius = kinematics_param.radius;
    double separation = kinematics_param.separation;
    double interval = kinematics_param.interval;
    uint16_t count_per_cycle = kinematics_param.count_per_cycle;

    double speed_right = (double)k_forward.speed[0] * 2 * M_PI /
                            (count_per_cycle * interval);
    double speed_left = (double)k_forward.speed[1] * 2 * M_PI /
                            (count_per_cycle * interval);

    if (kinematics_param.reverse[0])
        speed_right = -speed_right;
    if (kinematics_param.reverse[1])
        speed_left = -speed_left;

    k_forward.velocity.linear_x = (radius * (speed_left + speed_right)) / 2 *
                                      kinematics_param.linear_factor;
    k_forward.velocity.angular_z = (radius * (speed_right - speed_left)) / separation *
                                       kinematics_param.angular_factor;
}
