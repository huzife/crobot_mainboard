#include "bus_serial.h"
#include "los_task.h"
#include "main.h"
#include "print.h"

static UART_HandleTypeDef bus_serial0_uart;
static UART_HandleTypeDef bus_serial1_uart;

static struct Bus_Serial {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* txen_port;
    uint16_t txen_pin;
} bus_serial[BUS_NUM];

static void bus_serial_uart_setup(UART_HandleTypeDef* huart) {
    huart->Init.BaudRate = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_EVEN;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(huart) != HAL_OK)
        Error_Handler();
}

static void bus_serial0_setup() {
    // init Bus_Serial
    bus_serial[0].huart = &bus_serial0_uart;
    bus_serial[0].txen_port = NULL;
    bus_serial[0].txen_pin = 0;

    // msp init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // hal uart init
    bus_serial0_uart.Instance = UART5;
    bus_serial_uart_setup(&bus_serial0_uart);
}

static void bus_serial1_setup() {
    // init Bus_Serial
    bus_serial[1].huart = &bus_serial1_uart;
    bus_serial[1].txen_port = GPIOA;
    bus_serial[1].txen_pin = GPIO_PIN_15;

    // msp init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // hal uart init
    bus_serial1_uart.Instance = UART4;
    bus_serial_uart_setup(&bus_serial1_uart);
}

void bus_serial_init() {
    bus_serial0_setup();
    bus_serial1_setup();
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

    UART_HandleTypeDef* uart = bus_serial[id].huart;
    // PRINT("Bus %d state: %d\n", id, uart->gState);

    // lock bus
    LOS_TaskLock();

    // write
    uint32_t start = HAL_GetTick();
    start_transmission(id);
    if (HAL_UART_Transmit(uart, write_buf, write_len, timeout) != HAL_OK) {
        PRINT("Bus transmit error, id = %d\n", id);
        LOS_TaskUnlock();
        return false;
    }
    end_transmission(id);
    timeout -= (HAL_GetTick() - start);

    // read
    if (read_len && HAL_UART_Receive(uart, read_buf, read_len, timeout) != HAL_OK) {
        PRINT("Bus receive error, id = %d\n", id);
        LOS_TaskUnlock();
        return false;
    }

    // unlock bus
    LOS_TaskUnlock();

    return true;
}
