#ifndef USER_KINEMATICS_H
#define USER_KINEMATICS_H

#include "velocity.h"
#include <stdint.h>

#if defined LOSCFG_ROBOT_BASE_2WD
#define MOTOR_NUM 2
#endif

typedef struct {
    Velocity velocity;
    uint16_t speed[MOTOR_NUM];
} Kinematics;

typedef struct {
    float position_x;
    float position_y;
    float direction;
} Odometry;

void kinematics_update_odom(float interval);
Odometry kinematics_get_odom();

void kinematics_set_target_velocity(Velocity velocity);
Velocity kinematics_get_current_velocity();

void kinematics_set_current_motor_speed(uint16_t* speeds);
uint16_t* kinematics_get_target_motor_speed();

void kinematics_inverse();
void kinematics_forward();

#endif // USER_KINEMATICS_H
