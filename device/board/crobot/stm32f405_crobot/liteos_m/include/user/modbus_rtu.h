#ifndef USER_MODBUS_RTU_H
#define USER_MODBUS_RTU_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

bool modbus_rtu_get_input_regs(uint8_t bus_id,
                               uint8_t addr,
                               uint16_t start,
                               uint16_t len,
                               uint16_t* read_buf);

bool modbus_rtu_get_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t len,
                                 uint16_t* read_buf);

bool modbus_rtu_set_holding_reg(uint8_t bus_id,
                                uint8_t addr,
                                uint16_t reg,
                                uint16_t val);

bool modbus_rtu_set_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t* vals,
                                 uint16_t len);

#endif // USER_MODBUS_RTU_H
