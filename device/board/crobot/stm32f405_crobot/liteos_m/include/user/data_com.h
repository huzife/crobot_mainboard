#ifndef CONTROL_DATA_COM_H
#define CONTROL_DATA_COM_H

#include "stdint.h"

#define COM_DATA_TX_SIZE 64u

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
    uint8_t buf[64];   // 缓存地址
    uint8_t flag;      // 解析完成标志
    uint8_t FE;        // 帧头第一字节标志
    uint8_t FH;        // 接收完整帧头标志
} Data_Parser;

void data_parser_init(Data_Parser *parser); // 解析器初始化函数
void parse(Data_Parser *parser, uint8_t data); // 解析数据函数
void process_data(uint8_t *buf); // 帧数据处理函数

#endif  // CONTROL_DATA_COM_H
