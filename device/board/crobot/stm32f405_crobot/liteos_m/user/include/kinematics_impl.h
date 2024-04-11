#ifndef USER_KINEMATICS_IMPL_H
#define USER_KINEMATICS_IMPL_H

#include "odometry.h"
#include "velocity.h"

#if defined LOSCFG_ROBOT_BASE_2WD
#define WHEEL_NUM 2
#elif defined LOSCFG_ROBOT_BASE_3WO
#define WHEEL_NUM 3
#elif defined LOSCFG_ROBOT_BASE_4WD
#define WHEEL_NUM 4
#endif

/// @brief Update odometry
void kinematics_update_odometry(Odometry* odometry, Velocity velocity, double dt);

/// @brief Calculate target motor speed
void kinematics_inverse_func(Velocity velocity, double speeds[]);

/// @brief Calculate current motor velocity
void kinematics_forward_func(double speeds[], Velocity* velocity);

#endif // USER_KINEMATICS_IMPL_H
