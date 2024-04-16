#ifndef USER_ICM42605_H
#define USER_ICM42605_H

#include <stdint.h>

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_x;
    float angular_y;
    float angular_z;
} IMU_Data;

/// @brief Init icm42605
void icm42605_init();

/// @brief Update IMU temperature
void icm42605_update_temperature();

/// @brief Update IMU raw data
void icm42605_update_data();

/// @brief Get temperature
/// @return float
float icm42605_get_temperature();

/// @brief Get raw data
/// @return IMU_Data
IMU_Data icm42605_get_data();

#endif // USER_ICM42605_H
