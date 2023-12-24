#ifndef CORE_SHARE_WARE_H
#define CORE_SHARE_WARE_H

// common
#include "data_com.h"
#include "icm42605.h"
#include "s_queue.h"

#include "stm32f4xx_hal.h"

#include <math.h>

// 浮点数-十六进制转换
typedef union {
    uint8_t hex[4];
    float float_value;
} FloatHexUnion;

// 共享变量
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

extern uint8_t com_tx_data[COM_DATA_TX_SIZE];  // uart通信发送缓冲
extern uint8_t com_rx_data;

// extern uint32_t message_queue; // 消息队列句柄
extern SQueue data_queue; // 环形队列

// 共享结构体
extern Data_Parser parser;          // 数据解析器结构体
extern ICM_Raw_Data icm_raw_data;   // ICM42605 原始数据结构体

#endif // CORE_SHARE_WARE_H
