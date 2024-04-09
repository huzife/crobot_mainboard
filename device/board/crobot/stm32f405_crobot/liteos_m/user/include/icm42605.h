#ifndef USER_ICM42605_H
#define USER_ICM42605_H

#include <stdint.h>

typedef struct {
    volatile float accel_x;
    volatile float accel_y;
    volatile float accel_z;
    volatile float angular_x;
    volatile float angular_y;
    volatile float angular_z;
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
