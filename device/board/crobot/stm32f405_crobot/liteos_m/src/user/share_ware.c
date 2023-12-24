#include "share_ware.h"

// 共享变量
uint8_t com_tx_data[COM_DATA_TX_SIZE];  // uart通信发送缓冲
uint8_t com_rx_data;

// uint32_t message_queue; // 消息队列句柄
SQueue data_queue; // 环形队列

// 共享结构体
Data_Parser parser;
ICM_Raw_Data icm_raw_data;
