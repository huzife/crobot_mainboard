#include "ultrasonic.h"
#include "modbus_rtu.h"

static uint16_t ultrasonic_range;

bool ultrasonic_init() {
    ultrasonic_range = 0xFFFF;
    return modbus_get_input_regs(1, 1, 0, 1, &ultrasonic_range);
}

void ultrasonic_update_range() {
    modbus_get_input_regs(1, 1, 0, 1, &ultrasonic_range);
}

uint16_t ultrasonic_get_range() {
    return ultrasonic_range;
}
