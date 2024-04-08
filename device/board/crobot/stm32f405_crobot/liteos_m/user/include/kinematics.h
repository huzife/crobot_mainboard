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

/// @brief Init kinematics mutex
void kinematics_init();

/// @brief Update wheel odometry
void kinematics_update_odom();

/// @brief Get current wheel odometry
/// @return Odometry
Odometry kinematics_get_odom();

/// @brief Set target velocity
/// @param[in] velocity Target velocity
void kinematics_set_target_velocity(Velocity velocity);

/// @brief Get current velocity
/// @return Velocity
Velocity kinematics_get_current_velocity();

/// @brief Set current motor speed
/// @param[in] speeds An array that contains each motor's speed
void kinematics_set_current_motor_speed(int16_t* speeds);

/// @brief Get target motor speed
/// @return int16_t*, An array that contains each motor's speed
int16_t* kinematics_get_target_motor_speed();

/// @brief Calculate target motor speed
void kinematics_inverse();

/// @brief Calculate current motor velocity
void kinematics_forward();

#endif // USER_KINEMATICS_H
