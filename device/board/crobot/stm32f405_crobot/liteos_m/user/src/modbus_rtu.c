#include "modbus_rtu.h"
#include "los_debug.h"
#include "los_mux.h"
#include <string.h>

#define BUS_NUM 3
#define BUF_SIZE 64

typedef enum {
    GET_HOLDING_REG = 0x03,
    GET_INPUT_REG = 0x04,
    SET_HOLDING_REG = 0x06,
    SET_HOLDING_REGS = 0x10
} Modbus_Function_Code;

typedef enum {
    BUS_READY,
    BUS_WAITING,
    BUS_BUSY,
    BUS_TIMEOUT,
    BUS_ERROR
} Bus_Status;

static struct Bus {
    // hardware parameters
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef dma_tx;
    GPIO_TypeDef* txen_port;
    uint16_t txen_pin;
    TIM_HandleTypeDef tim;

    // request parameters
    uint32_t mtx;
    uint8_t write_buf[BUF_SIZE];
    uint8_t read_buf[BUF_SIZE];
    uint16_t read_len;  // Expected length to read
    uint16_t recv_len;  // Length of data already read
    volatile Bus_Status status;
} bus[BUS_NUM];

static void modbus_uart_init(UART_HandleTypeDef* huart) {
    huart->Init.BaudRate = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_EVEN;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(huart);
}

static void modbus_uart_dma_init(DMA_HandleTypeDef* hdma) {
    hdma->Init.Channel = DMA_CHANNEL_4;
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma->Init.Mode = DMA_NORMAL;
    hdma->Init.Priority = DMA_PRIORITY_LOW;
    hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(hdma);
}

static void modbus_t15_timer_init(TIM_HandleTypeDef* htim) {
    htim->Init.Prescaler = 84-1;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 750-1;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(htim);
}

static void bus0_init() {
    // init bus[0]
    LOS_MuxCreate(&bus[0].mtx);
    bus[0].txen_port = NULL;
    bus[0].txen_pin = 0;
    bus[0].status = BUS_READY;

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
    bus[0].uart.Instance = UART5;
    modbus_uart_init(&bus[0].uart);
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);

    // hal uart dma init
    __HAL_RCC_DMA1_CLK_ENABLE();
    bus[0].dma_tx.Instance = DMA1_Stream7;
    bus[0].dma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    modbus_uart_dma_init(&bus[0].dma_tx);
    __HAL_LINKDMA(&bus[0].uart, hdmatx, bus[0].dma_tx);
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

    // hal tim init
    __HAL_RCC_TIM10_CLK_ENABLE();
    bus[0].tim.Instance = TIM10;
    modbus_t15_timer_init(&bus[0].tim);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    __HAL_TIM_CLEAR_FLAG(&bus[0].tim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&bus[0].tim, TIM_IT_UPDATE);
}

static void bus1_init() {
    // init bus[1]
    LOS_MuxCreate(&bus[1].mtx);
    bus[1].txen_port = GPIOA;
    bus[1].txen_pin = GPIO_PIN_15;
    bus[1].status = BUS_READY;

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
    bus[1].uart.Instance = UART4;
    modbus_uart_init(&bus[1].uart);
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    // hal uart dma init
    __HAL_RCC_DMA1_CLK_ENABLE();
    bus[1].dma_tx.Instance = DMA1_Stream4;
    bus[1].dma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    modbus_uart_dma_init(&bus[1].dma_tx);
    __HAL_LINKDMA(&bus[1].uart, hdmatx, bus[1].dma_tx);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    // hal tim init
    __HAL_RCC_TIM11_CLK_ENABLE();
    bus[1].tim.Instance = TIM11;
    modbus_t15_timer_init(&bus[1].tim);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    __HAL_TIM_CLEAR_FLAG(&bus[1].tim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&bus[1].tim, TIM_IT_UPDATE);
}

void modbus_init() {
    bus0_init();
    bus1_init();
}

inline static void start_transmission(uint16_t id) {
    if (bus[id].txen_port && bus[id].txen_pin)
        HAL_GPIO_WritePin(bus[id].txen_port, bus[id].txen_pin, GPIO_PIN_SET);
}

inline static void end_transmission(uint16_t id) {
    if (bus[id].txen_port && bus[id].txen_pin)
        HAL_GPIO_WritePin(bus[id].txen_port, bus[id].txen_pin, GPIO_PIN_RESET);
}

static bool bus_request(uint16_t id, uint16_t write_len,
                        uint16_t read_len, uint32_t timeout) {
    if (id >= BUS_NUM)
        return false;

    uint32_t start_tick = LOS_TickCountGet();

    // lock bus
    uint32_t ret = LOS_MuxPend(bus[id].mtx, timeout);
    if (ret != LOS_OK) {
        PRINT_ERR("Bus request failed, didn't get mutex, id = %d, ret = %x\n", id, ret);
        return false;
    }

    // wait for T3.5 timer
    while (bus[id].status != BUS_READY) {}
    bus[id].status = BUS_BUSY;
    bus[id].read_len = read_len;
    bus[id].recv_len = 0;

    // start request
    start_transmission(id);
    if (HAL_UART_Transmit_DMA(&bus[id].uart, bus[id].write_buf, write_len) != HAL_OK) {
        PRINT_ERR("Bus transmit failed, id = %d\n", id);
        bus[id].status = BUS_READY;
        end_transmission(id);
        LOS_MuxPost(bus[id].mtx);
        return false;
    }

    // block until request finished or timeout
    while (bus[id].status == BUS_BUSY) {
        if (LOS_TickCountGet() - start_tick > timeout) {
            HAL_UART_Abort(&bus[id].uart);
            __HAL_TIM_DISABLE(&bus[id].tim);
            bus[id].status = BUS_TIMEOUT;
            break;
        }
    }

    // unlock bus
    LOS_MuxPost(bus[id].mtx);

    // check bus status
    switch (bus[id].status) {
        case BUS_READY:
        case BUS_WAITING:
            return true;

        case BUS_BUSY:
            PRINT_ERR("Bus request reach impossible branch??? Please check!!!\n");
            return false;

        case BUS_TIMEOUT:
            PRINT_ERR("Bus request timeout, id = %d\n", id);
            bus[id].status = BUS_READY;
            return false;

        case BUS_ERROR:
            PRINT_ERR("Bus error, byte rx interval excceeded T1.5, id = %d\n", id);
            bus[id].status = BUS_READY;
            return false;
    }
}

inline static void modbus_timer_reset(uint16_t id) {
    __HAL_TIM_SET_COUNTER(&bus[id].tim, 0);
    __HAL_TIM_CLEAR_FLAG(&bus[id].tim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&bus[id].tim);
}

void modbus_uart_callback(uint16_t id) {
    UART_HandleTypeDef* uart = &bus[id].uart;


    // transmit
    if (__HAL_UART_GET_IT_SOURCE(uart, UART_IT_TC) &&
        __HAL_UART_GET_FLAG(uart, UART_FLAG_TC)) {
        end_transmission(id);
        HAL_UART_IRQHandler(uart); // TC flag will be cleared here
        if (bus[id].read_len) {
            // enable uart RXNE interrupt
            __HAL_UART_ENABLE_IT(uart, UART_IT_RXNE);

            // set T1.5 timer ARR
            __HAL_TIM_SET_AUTORELOAD(&bus[id].tim, 750 - 1);
        }
    }

    // receive
    if (__HAL_UART_GET_IT_SOURCE(uart, UART_IT_RXNE) &&
        __HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE)) {
        // RXNE flag will be cleared by reading DR
        bus[id].read_buf[bus[id].recv_len++] = uart->Instance->DR & 0xFF;

        if (bus[id].recv_len == bus[id].read_len) {
            // receive expected data, disable RXNE interrupt
            __HAL_UART_DISABLE_IT(uart, UART_IT_RXNE);

            // set t35 timer ARR
            __HAL_TIM_SET_AUTORELOAD(&bus[id].tim, 1750 - 1);
            bus[id].status = BUS_WAITING;
        }
        modbus_timer_reset(id);
    }
}

void modbus_dma_tx_callback(uint16_t id) {
    HAL_DMA_IRQHandler(&bus[id].dma_tx);
}

void modbus_timer_timeout_callback(uint16_t id) {
    if (__HAL_TIM_GET_IT_SOURCE(&bus[id].tim, TIM_IT_UPDATE) &&
        __HAL_TIM_GET_FLAG(&bus[id].tim, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&bus[id].tim, TIM_FLAG_UPDATE);
        __HAL_TIM_DISABLE(&bus[id].tim);

        // check and change bus status
        if (bus[id].status == BUS_BUSY) {
            HAL_UART_Abort(&bus[id].uart);
            bus[id].status = BUS_ERROR;
        } else if (bus[id].status == BUS_WAITING) {
            bus[id].status = BUS_READY;
        }
    }
}

static uint16_t modbus_crc(const uint8_t* buf, uint8_t len) {
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

static bool modbus_get_regs(uint8_t bus_id,
                            uint8_t addr,
                            Modbus_Function_Code code,
                            uint16_t start,
                            uint16_t len,
                            uint16_t* reg_vals) {
    if (bus_id >= BUS_NUM || 2 * len + 5 > BUF_SIZE)
        return false;

    uint8_t* write_buf = bus[bus_id].write_buf;
    uint8_t* read_buf = bus[bus_id].read_buf;

    // send data
    write_buf[0] = addr;
    write_buf[1] = code;
    write_buf[2] = start >> 8;
    write_buf[3] = start & 0xFF;
    write_buf[4] = len >> 8;
    write_buf[5] = len & 0xFF;

    uint16_t crc = modbus_crc(write_buf, 6);
    write_buf[6] = crc & 0xFF;
    write_buf[7] = crc >> 8;

    uint16_t read_len = 2 * len + 5;
    if (!bus_request(bus_id, 8, read_len, 100))
        return false;

    crc = (read_buf[read_len - 1] << 8) | read_buf[read_len - 2];
    if (crc != modbus_crc(read_buf, read_len - 2))
        return false;

    if (read_buf[0] != addr ||
        read_buf[1] != code ||
        read_buf[2] != len * 2)
        return false;

    for (int i = 0; i < len; i++) {
        reg_vals[i] = ((read_buf[3 + i * 2] << 8) | read_buf[4 + i * 2]);
    }

    return true;
}

bool modbus_get_input_regs(uint8_t bus_id,
                           uint8_t addr,
                           uint16_t start,
                           uint16_t len,
                           uint16_t* reg_vals) {
    return modbus_get_regs(
               bus_id, addr, GET_INPUT_REG, start, len, reg_vals);
}

bool modbus_get_holding_regs(uint8_t bus_id,
                             uint8_t addr,
                             uint16_t start,
                             uint16_t len,
                             uint16_t* reg_vals) {
    return modbus_get_regs(
               bus_id, addr, GET_HOLDING_REG, start, len, reg_vals);
}

bool modbus_set_holding_reg(uint8_t bus_id,
                            uint8_t addr,
                            uint16_t reg,
                            uint16_t val) {
    if (bus_id >= BUS_NUM)
        return false;

    uint8_t* write_buf = bus[bus_id].write_buf;
    uint8_t* read_buf = bus[bus_id].read_buf;

    // send data
    write_buf[0] = addr;
    write_buf[1] = SET_HOLDING_REG;
    write_buf[2] = reg >> 8;
    write_buf[3] = reg & 0xFF;
    write_buf[4] = val >> 8;
    write_buf[5] = val & 0xFF;

    uint16_t crc = modbus_crc(write_buf, 6);
    write_buf[6] = crc & 0xFF;
    write_buf[7] = crc >> 8;

    uint16_t read_len = addr ? 8 : 0;
    if (!bus_request(bus_id, 8, read_len, 100))
        return false;

    if (!addr)
        return true;

    return memcmp(write_buf, read_buf, 8) == 0;
}

bool modbus_set_holding_regs(uint8_t bus_id,
                             uint8_t addr,
                             uint16_t start,
                             uint16_t* vals,
                             uint16_t len) {
    if (bus_id >= BUS_NUM || 2 * len + 9 > BUF_SIZE)
        return false;

    uint8_t* write_buf = bus[bus_id].write_buf;
    uint8_t* read_buf = bus[bus_id].read_buf;

    write_buf[0] = addr;
    write_buf[1] = SET_HOLDING_REGS;
    write_buf[2] = start >> 8;
    write_buf[3] = start & 0xFF;
    write_buf[4] = len >> 8;
    write_buf[5] = len & 0xFF;
    write_buf[6] = len * 2;

    uint16_t idx = 7;
    for (int i = 0; i < len; i++) {
        write_buf[idx++] = vals[i] >> 8;
        write_buf[idx++] = vals[i] & 0xFF;
    }

    uint16_t crc = modbus_crc(write_buf, idx);
    write_buf[idx++] = crc & 0xFF;
    write_buf[idx++] = crc >> 8;

    uint16_t read_len = addr ? 8 : 0;
    if (!bus_request(bus_id, 9 + len * 2, read_len, 100))
        return false;

    if (!addr)
        return true;

    if (memcmp(write_buf, read_buf, 6))
        return false;

    crc = modbus_crc(read_buf, 6);
    if (crc != ((read_buf[7] << 8) | read_buf[6]))
        return false;

    return true;
}
