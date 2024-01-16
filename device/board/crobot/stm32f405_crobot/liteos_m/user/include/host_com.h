#ifndef USER_HOST_COM_H
#define USER_HOST_COM_H

#include "los_event.h"
#include <stdbool.h>
#include <stdint.h>

#define HOST_COM_TX_DONE 0x01

extern EVENT_CB_S host_com_event;

// 数据帧功能码
typedef enum {
    NONE = 0,
    SET_SPEED,
    GET_SPEED,
    GET_IMU_TEMPERATURE,
    GET_IMU_DATA
} Function_Code;

void host_com_init(uint8_t* pool, uint16_t buf_len); // 解析器初始化函数
bool host_com_parse(); // 解析数据函数
void host_com_process(); // 帧数据处理函数

#endif  // USER_HOST_COM_H
