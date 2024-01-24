#ifndef USER_MODBUS_RTU_H
#define USER_MODBUS_RTU_H

#include <stdbool.h>
#include <stdint.h>


/// @brief Get input registers by modbus rtu protocol
/// @param[in] bus_id Bus id
/// @param[in] addr Slave device address
/// @param[in] start Start register
/// @param[in] len Number of registers to read
/// @param[out] read_buf Pointer to read buffer
/// @return bool
bool modbus_rtu_get_input_regs(uint8_t bus_id,
                               uint8_t addr,
                               uint16_t start,
                               uint16_t len,
                               uint16_t* read_buf);

/// @brief Get holding registers by modbus rtu protocol
/// @param[in] bus_id Bus id
/// @param[in] addr Slave device address
/// @param[in] start Start register
/// @param[in] len Number of registers to read
/// @param[out] read_buf Pointer to read buffer
/// @return bool
bool modbus_rtu_get_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t len,
                                 uint16_t* read_buf);

/// @brief set holding register by modbus rtu protocol
/// @param[in] bus_id Bus id
/// @param[in] addr Slave device address
/// @param[in] reg Register to write
/// @param[in] val Value to write
/// @return bool
bool modbus_rtu_set_holding_reg(uint8_t bus_id,
                                uint8_t addr,
                                uint16_t reg,
                                uint16_t val);

/// @brief set holding registers by modbus rtu protocol
/// @param[in] bus_id Bus id
/// @param[in] addr Slave device address
/// @param[in] start Start register
/// @param[in] vals Pointer to values
/// @param[in] len Number of registers to write
/// @return bool
bool modbus_rtu_set_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t* vals,
                                 uint16_t len);

#endif // USER_MODBUS_RTU_H
