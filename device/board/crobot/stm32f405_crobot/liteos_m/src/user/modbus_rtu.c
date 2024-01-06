#include "modbus_rtu.h"
#include "bus_serial.h"
#include <stdint.h>
#include <string.h>

#define BUF_SIZE 128
static uint8_t bus_write_buf[BUF_SIZE];
static uint8_t bus_read_buf[BUF_SIZE];

typedef enum {
    GET_HOLDING_REG = 0x03,
    GET_INPUT_REG = 0x04,
    SET_HOLDING_REG = 0x06,
    SET_HOLDING_REGS = 0x10
} Modbus_Function_Code;

static uint16_t modbus_rtu_crc(const uint8_t* buf, uint8_t len) {
    unsigned int temp, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < len; i++) {
        temp = temp ^ buf[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    temp &= 0xFFFF;
    return temp;
}

static bool modbus_rtu_get_regs(uint8_t bus_id,
                                uint8_t addr,

                                Modbus_Function_Code code,
                                uint16_t start,
                                uint16_t len,
                                uint16_t* read_buf) {
    if (bus_id >= BUS_NUM || 2 * len + 5 > BUF_SIZE)
        return false;

    // send data
    bus_write_buf[0] = addr;
    bus_write_buf[1] = code;
    bus_write_buf[2] = start >> 8;
    bus_write_buf[3] = start & 0xFF;
    bus_write_buf[4] = len >> 8;
    bus_write_buf[5] = len & 0xFF;

    uint16_t crc = modbus_rtu_crc(bus_write_buf, 6);
    bus_write_buf[6] = crc & 0xFF;
    bus_write_buf[7] = crc >> 8;

    if (!bus_serial_write(bus_id, bus_write_buf, 8))
        return false;

    // receive data
    uint16_t recv_len = 2 * len + 5;
    if (!bus_serial_read(bus_id, bus_read_buf, recv_len))
        return false;

    crc = (bus_read_buf[recv_len - 1] << 8) | bus_read_buf[recv_len - 2];
    if (crc != modbus_rtu_crc(bus_read_buf, recv_len - 2))
        return false;

    if (bus_read_buf[0] != addr ||
        bus_read_buf[1] != code ||
        bus_read_buf[2] != len * 2)
        return false;

    for (int i = 0; i < len; i++) {
        read_buf[i] = ((bus_read_buf[3 + i * 2] << 8) |
                       bus_read_buf[4 + i * 2]);
    }

    return true;
}

bool modbus_rtu_get_input_regs(uint8_t bus_id,
                               uint8_t addr,
                               uint16_t start,
                               uint16_t len,
                               uint16_t* read_buf) {
    return modbus_rtu_get_regs(
               bus_id, addr, GET_INPUT_REG, start, len, read_buf);
}

bool modbus_rtu_get_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t len,
                                 uint16_t* read_buf) {
    return modbus_rtu_get_regs(
               bus_id, addr, GET_HOLDING_REG, start, len, read_buf);
}

bool modbus_rtu_set_holding_reg(uint8_t bus_id,
                                uint8_t addr,
                                uint16_t reg,
                                uint16_t val) {
    if (bus_id >= BUS_NUM)
        return false;

    // send data
    bus_write_buf[0] = addr;
    bus_write_buf[1] = SET_HOLDING_REG;
    bus_write_buf[2] = reg >> 8;
    bus_write_buf[3] = reg & 0xFF;
    bus_write_buf[4] = val >> 8;
    bus_write_buf[5] = val & 0xFF;

    uint16_t crc = modbus_rtu_crc(bus_write_buf, 6);
    bus_write_buf[6] = crc & 0xFF;
    bus_write_buf[7] = crc >> 8;

    if (!bus_serial_write(bus_id, bus_write_buf, 8))
        return false;

    // receive data
    if (!addr)
        return true;

    if (!bus_serial_read(bus_id, bus_read_buf, 8))
        return false;

    return memcmp(bus_write_buf, bus_read_buf, 8) == 0;
}

bool modbus_rtu_set_holding_regs(uint8_t bus_id,
                                 uint8_t addr,
                                 uint16_t start,
                                 uint16_t* vals,
                                 uint16_t len) {
    if (bus_id >= BUS_NUM || 2 * len + 9 > BUF_SIZE)
        return false;

    bus_write_buf[0] = addr;
    bus_write_buf[1] = SET_HOLDING_REGS;
    bus_write_buf[2] = start >> 8;
    bus_write_buf[3] = start & 0xFF;
    bus_write_buf[4] = len >> 8;
    bus_write_buf[5] = len & 0xFF;
    bus_write_buf[6] = len * 2;

    uint16_t idx = 7;
    for (int i = 0; i < len; i++) {
        bus_write_buf[idx++] = vals[i] >> 8;
        bus_write_buf[idx++] = vals[i] & 0xFF;
    }

    uint16_t crc = modbus_rtu_crc(bus_write_buf, idx);
    bus_write_buf[idx++] = crc & 0xFF;
    bus_write_buf[idx++] = crc >> 8;

    if (!bus_serial_write(bus_id, bus_write_buf, 9 + len * 2))
        return false;

    // receive data
    if (!addr)
        return true;

    if (!bus_serial_read(bus_id, bus_read_buf, 8))
        return false;

    if (memcmp(bus_write_buf, bus_read_buf, 6))
        return false;

    crc = modbus_rtu_crc(bus_read_buf, 6);
    if (crc != ((bus_read_buf[7] << 8) | bus_read_buf[6]))
        return false;

    return true;
}
