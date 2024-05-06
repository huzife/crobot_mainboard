#ifndef USER_KINEMATICS_H
#define USER_KINEMATICS_H

#include "odometry.h"
#include "velocity.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    Velocity velocity;
    int16_t* speeds;
} Kinematics;

typedef enum {
    KINEMATICS_ROBOT_BASE_2WD,
    KINEMATICS_ROBOT_BASE_3WO,
    KINEMATICS_ROBOT_BASE_4WD,
    KINEMATICS_ROBOT_BASE_4MEC
} Kinematics_Robot_Base;

/// @brief Init kinematics mutex
void kinematics_init();

bool kinematics_set_robot_base(Kinematics_Robot_Base type, void* params, uint32_t size);

/// @brief Get current wheel odometry
void kinematics_get_odometry_and_velocity(Odometry* odom, Velocity* vel);

/// @brief Reset odometry
void kinematics_reset_odometry();

/// @brief Set linear and angular correction factor
void kinematics_set_correction_factor(float linear_x, float linear_y, float angular);

/// @brief Set target velocity
/// @param[in] velocity Target velocity
void kinematics_set_target_velocity(Velocity velocity);

/// @brief Handle velocity and send motor speed
void kinematics_handle_velocity();

/// @brief Update velocity and odom info
void kinematics_update_info();

bool kinematics_start();

void kinematics_end();

#endif // USER_KINEMATICS_H
