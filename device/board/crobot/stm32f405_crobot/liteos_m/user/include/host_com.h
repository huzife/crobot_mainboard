#ifndef USER_HOST_COM_H
#define USER_HOST_COM_H

#include <stdbool.h>
#include <stdint.h>

#define HOST_COM_TX_DONE 0x01

/// @brief Init host com module
/// @param[in] buf_len Buffer lenght of host com
bool host_com_init(uint16_t buf_len);

/// @brief Try to receive a byte and parse it using state machine
/// @return bool
/// @retval true: Succeed received and parse a byte
/// @retval false: No data avaliable
bool host_com_parse();

#endif  // USER_HOST_COM_H
