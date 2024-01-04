#include "bus_serial.h"

Bus_Serial bus_serial[BUS_NUM];

bool bus_serial_init(uint16_t bus_id, UART_HandleTypeDef* huart,
                     GPIO_TypeDef* txen_port, uint16_t txen_pin) {
    if (bus_id >= BUS_NUM || huart == NULL)
        return false;

    bus_serial[bus_id].huart = huart;
    bus_serial[bus_id].txen_port = txen_port;
    bus_serial[bus_id].txen_pin = txen_pin;

    return true;
}

bool bus_serial_read(uint16_t bus_id, uint8_t* buf, uint16_t len) {
    if (bus_id >= BUS_NUM || bus_serial[bus_id].huart == NULL)
        return false;

    HAL_UART_Receive(bus_serial[bus_id].huart, buf, len, 500);

    return true;
}

bool bus_serial_write(uint16_t bus_id, uint8_t* buf, uint16_t len) {
    if (bus_id >= BUS_NUM || bus_serial[bus_id].huart == NULL)
        return false;

    GPIO_TypeDef* port = bus_serial->txen_port;
    uint16_t pin = bus_serial->txen_pin;
    if (port && pin)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    HAL_UART_Transmit(bus_serial[bus_id].huart, buf, len, 500);

    if (port && pin)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    return true;
}
