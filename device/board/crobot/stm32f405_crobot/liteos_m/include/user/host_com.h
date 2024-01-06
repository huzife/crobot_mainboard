#ifndef USER_HOST_COM_H
#define USER_HOST_COM_H

#include "s_queue.h"
#include "los_event.h"
#include <stdint.h>

#define HOST_TX_BUF_SIZE 64
#define HOST_COM_TX_DONE 0x01

extern uint8_t host_tx_buf[HOST_TX_BUF_SIZE];  // host通信发送缓冲
extern uint8_t host_rx_data;
extern SQueue host_rx_queue;
extern EVENT_CB_S host_com_event;

// 数据帧功能码
typedef enum {
    NONE = 0,
    SET_SPEED,
    GET_SPEED,
    GET_IMU_TEMPERATURE,
    GET_IMU
} Function_Code;

// 数据解析结构体
typedef struct {
    uint16_t buf_len;  // 缓存长度
    uint16_t data_len; // 当前数据长度
    uint8_t* buf;      // 缓存地址
    uint8_t flag;      // 解析完成标志
    uint8_t FE;        // 帧头第一字节标志
    uint8_t FH;        // 接收完整帧头标志
} Host_Parser;

void host_parser_init(Host_Parser *parser, uint8_t* pool, uint16_t buf_len); // 解析器初始化函数
void parse(Host_Parser *parser, uint8_t data); // 解析数据函数
void process_data(uint8_t *buf); // 帧数据处理函数

#endif  // USER_HOST_COM_H
