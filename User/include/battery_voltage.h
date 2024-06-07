#ifndef USER_BATTERY_VOLTAGE_H
#define USER_BATTERY_VOLTAGE_H

/// @brief Init battery voltage adc
void battery_voltage_init();

/// @brief Get battery voltage
/// @return float
float battery_get_voltage();

/// @brief Update battery voltage
void battery_voltage_update();

#endif // USER_BATTERY_VOLTAGE_H
