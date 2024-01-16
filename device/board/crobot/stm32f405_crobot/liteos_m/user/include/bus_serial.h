#ifndef USER_BUS_SERIAL_H
#define USER_BUS_SERIAL_H

#include "usart.h"
#include <stdbool.h>

#define BUS_NUM 1

bool bus_serial_init(uint16_t id, UART_HandleTypeDef* huart,
                     GPIO_TypeDef* txen_port, uint16_t txen_pin);
bool bus_serial_request(uint16_t id, uint8_t* write_buf, uint16_t write_len,
                        uint8_t* read_buf, uint16_t read_len, uint32_t timeout);

#endif // USER_BUS_SERIAL_H
