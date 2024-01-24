#include "bus_serial.h"
#include "los_mux.h"

static struct {
    uint32_t mtx;
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* txen_port;
    uint16_t txen_pin;
} bus_serial[BUS_NUM];

bool bus_serial_init(uint16_t id, UART_HandleTypeDef* huart,
                     GPIO_TypeDef* txen_port, uint16_t txen_pin) {
    if (id >= BUS_NUM || huart == NULL)
        return false;

    LOS_MuxCreate(&bus_serial[id].mtx);
    bus_serial[id].huart = huart;
    bus_serial[id].txen_port = txen_port;
    bus_serial[id].txen_pin = txen_pin;

    return true;
}

inline static void start_transmission(uint16_t id) {
    if (bus_serial[id].txen_port && bus_serial[id].txen_pin)
        HAL_GPIO_WritePin(bus_serial[id].txen_port,
                          bus_serial[id].txen_pin,
                          GPIO_PIN_SET);
}

inline static void end_transmission(uint16_t id) {
    if (bus_serial[id].txen_port && bus_serial[id].txen_pin)
        HAL_GPIO_WritePin(bus_serial[id].txen_port,
                          bus_serial[id].txen_pin,
                          GPIO_PIN_RESET);
}

bool bus_serial_request(uint16_t id, uint8_t* write_buf, uint16_t write_len,
                        uint8_t* read_buf, uint16_t read_len, uint32_t timeout) {
    if (id >= BUS_NUM || bus_serial[id].huart == NULL)
        return false;

    uint32_t mtx = bus_serial[id].mtx;
    UART_HandleTypeDef* uart = bus_serial[id].huart;

    // lock bus
    if (LOS_MuxPend(mtx, 100) != LOS_OK)
        return false;

    // write
    start_transmission(id);
    if (HAL_UART_Transmit(uart, write_buf, write_len, timeout) != HAL_OK) {
        LOS_MuxPost(mtx);
        return false;
    }
    end_transmission(id);

    // read
    if (read_len && HAL_UART_Receive(uart, read_buf, read_len, timeout) != HAL_OK) {
        LOS_MuxPost(mtx);
        return false;
    }

    // unlock ubs
    LOS_MuxPost(mtx);

    return true;
}
