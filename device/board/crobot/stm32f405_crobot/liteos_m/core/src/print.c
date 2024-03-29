#include "print.h"
#include "main.h"

static UART_HandleTypeDef debug_uart;

void print_init() {
#ifndef ENABLE_PRINT
    return;
#endif

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    debug_uart.Instance = USART1;
    debug_uart.Init.BaudRate = 115200;
    debug_uart.Init.WordLength = UART_WORDLENGTH_8B;
    debug_uart.Init.StopBits = UART_STOPBITS_1;
    debug_uart.Init.Parity = UART_PARITY_NONE;
    debug_uart.Init.Mode = UART_MODE_TX_RX;
    debug_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    debug_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&debug_uart) != HAL_OK)
        Error_Handler();
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&debug_uart, (uint8_t*)&ch, 1, 100);
    return ch;
}
