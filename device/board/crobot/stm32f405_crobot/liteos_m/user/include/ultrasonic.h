#ifndef USER_ULTRASONIC_H
#define USER_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

/// @brief Init ultrasonic timer
/// @return bool
bool ultrasonic_init();

/// @brief Update ultrasonic range
void ultrasonic_update_range();

/// @brief Get ultrasonic range
/// @return uint16_t
uint16_t ultrasonic_get_range();

#endif // USER_ULTRASONIC_H
