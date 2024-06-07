#ifndef USER_ULTRASONIC_H
#define USER_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

/// @brief Init ultrasonic timer
bool ultrasonic_init();

/// @brief Update ultrasonic range
void ultrasonic_update_range();

/// @brief Get ultrasonic range
/// @return uint32_t
uint16_t ultrasonic_get_range();

#endif // USER_ULTRASONIC_H
