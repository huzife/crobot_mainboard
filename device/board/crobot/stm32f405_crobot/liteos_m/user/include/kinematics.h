#ifndef USER_KINEMATICS_H
#define USER_KINEMATICS_H

#include "velocity.h"
#include <stdint.h>

#if defined LOSCFG_ROBOT_BASE_2WD
#define WHEEL_NUM 2
#elif defined LOSCFG_ROBOT_BASE_4WD
#define WHEEL_NUM 4
#elif defined LOSCFG_ROBOT_BASE_3WO
#define WHEEL_NUM 3
#endif

extern uint32_t kinematics_task_id;

typedef struct {
    Velocity velocity;
    int16_t speed[WHEEL_NUM];
} Kinematics;

typedef struct {
    float position_x;
    float position_y;
    float direction;
} Odometry;

void kinematics_update_odom();
Odometry kinematics_get_odom();

void kinematics_set_target_velocity(Velocity velocity);
Velocity kinematics_get_current_velocity();

void kinematics_set_current_motor_speed(int16_t* speeds);
int16_t* kinematics_get_target_motor_speed();

void kinematics_inverse();
void kinematics_forward();

#endif // USER_KINEMATICS_H
