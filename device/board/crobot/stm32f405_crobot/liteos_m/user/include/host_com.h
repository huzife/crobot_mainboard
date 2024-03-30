#ifndef USER_HOST_COM_H
#define USER_HOST_COM_H

#include "vel_mux.h"
#include "los_event.h"
#include <stdbool.h>
#include <stdint.h>

#define HOST_COM_TX_DONE 0x01

extern uint32_t host_com_task_id;
extern EVENT_CB_S host_com_event;
extern Velocity_Message host_com_velocity;

// 数据帧功能码
typedef enum {
    NONE = 0,
    SET_VELOCITY,
    GET_ODOM,
    GET_IMU_TEMPERATURE,
    GET_IMU_DATA,
    GET_ULTRASONIC_RANGE
} Function_Code;

/// @brief Init host com module
/// @param[in] pool Address of memory pool
/// @param[in] buf_len Buffer lenght of host com
void host_com_init(uint8_t* pool, uint16_t buf_len);

/// @brief Try to receive a byte and parse it using state machine
/// @return bool
/// @retval true Succeed parsing a complete data frame
/// @retval false No data avaliable or a complete data frame has not been parsed yet
bool host_com_parse();

/// @brief Process data frame
void host_com_process();

#endif  // USER_HOST_COM_H
