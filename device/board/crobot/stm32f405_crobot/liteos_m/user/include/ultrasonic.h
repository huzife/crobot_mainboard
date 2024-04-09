#ifndef USER_ULTRASONIC_H
#define USER_ULTRASONIC_H

#include <stdint.h>

/// @brief Init ultrasonic timer
void ultrasonic_init();

/// @brief Update ultrasonic range
void ultrasonic_update_range();

/// @brief Get ultrasonic range
/// @return uint32_t
uint32_t ultrasonic_get_range();

#endif // USER_ULTRASONIC_H
