#include "ultrasonic.h"
#include "modbus_rtu.h"
#include "los_atomic.h"

static uint32_t ultrasonic_range;

void ultrasonic_init() {
    ultrasonic_range = 0xFFFF;
}

void ultrasonic_update_range() {
    uint16_t range;
    modbus_get_input_regs(1, 1, 0, 1, &range);
    LOS_AtomicSet((Atomic*)&ultrasonic_range, range);
}

uint32_t ultrasonic_get_range() {
    return LOS_AtomicRead((const Atomic*)&ultrasonic_range);
}
