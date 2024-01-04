#ifndef USER_BUS_SERIAL_H
#define USER_BUS_SERIAL_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define BUS_NUM 1

typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* txen_port;
    uint16_t txen_pin;
} Bus_Serial;

bool bus_serial_init(uint16_t bus_id, UART_HandleTypeDef* huart,
                     GPIO_TypeDef* txen_port, uint16_t txen_pin);
bool bus_serial_read(uint16_t bus_id, uint8_t* buf, uint16_t len);
bool bus_serial_write(uint16_t bus_id, uint8_t* buf, uint16_t len);

#endif // USER_BUS_SERIAL_H
