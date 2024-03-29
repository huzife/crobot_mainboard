#ifndef USER_BUS_SERIAL_H
#define USER_BUS_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

#define BUS_NUM 3

/// @brief Init bus serial
void bus_serial_init();

/// @brief Make a bus serial request
/// @param[in] id Bus id
/// @param[in] write_buf Ptr to write buffer
/// @param[in] write_len Length of data to write
/// @param[out] read_buf Ptr to read buffer
/// @param[in] read_len Length of data to read
/// @param[in] timeout Time limit
/// @return bool
/// @retval true Succeed writing and reading
/// @retval false Failed to write or read(timeout)
bool bus_serial_request(uint16_t id, uint8_t* write_buf, uint16_t write_len,
                        uint8_t* read_buf, uint16_t read_len, uint32_t timeout);

#endif // USER_BUS_SERIAL_H
