#ifndef USER_BUS_SERIAL_H
#define USER_BUS_SERIAL_H

#include "usart.h"
#include <stdbool.h>

#define BUS_NUM 3

/// @brief Init bus serial by id
/// @param[in] id Bus id
/// @param[in] huart Uart port
/// @param[in] txen_port TX enable port
/// @param[in] txen_pin TX enable pin
/// @return bool
/// @retval true Succeed initializing
/// @retval false Failed to initialize
bool bus_serial_init(uint16_t id, UART_HandleTypeDef* huart,
                     GPIO_TypeDef* txen_port, uint16_t txen_pin);

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
