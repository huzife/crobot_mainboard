#include "data_com.h"
#include "share_ware.h"

#include "los_memory.h"

/**
 *  函数功能：计算累加校验和
 *  入口参数：数据首地址、数据长度
 *  返 回 值：校验和
 */
uint8_t check_sum(uint8_t* data, uint32_t len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }

    return ~sum;
}

/**
 *  函数功能：数据解析器初始化
 *  入口参数：数据解析器结构体指针、缓冲区长度
 *  返 回 值：None
 */
void data_parser_init(Data_Parser *parser) {
    parser->flag = 0;       // 解析完成标志位置零
    parser->FE = 0;         // 帧头FE标志位置零
    parser->FH = 0;         // 完整帧头标志位置零
    parser->buf_len = 64;   // 缓冲区长度
    parser->data_len = 0;   // 当前数据长度置零
}

/**
 *  函数功能：解析数据
 *  入口参数：数据解析器结构体指针、一个字节数据
 *  返 回 值：None
 */
void parse(Data_Parser *parser, uint8_t data) {
    // 未完成则继续解析数据
    if (!parser->flag) {
        // 存入当前数据
        parser->buf[parser->data_len] = data;

        switch(data) {
            case 0xFE: {
                parser->FE = 1;
                break;
            }
            case 0xEF: {
                if (parser->FE) {
                    parser->FE = 0; // 清除FE标志
                    parser->FH = 1; // 完整帧头
                    parser->buf[0] = 0xFE;
                    parser->buf[1] = 0xEF;
                    parser->buf[2] = parser->buf_len;
                    parser->data_len = 1;
                }
                break;
            }
            default: {
                parser->FE = 0;
                break;
            }
        }
    }

    // 判断是否接收到完整的一帧数据
    if (parser->FH && (parser->data_len > parser->buf[2] + 2)) {
        // 累加法计算校验和
        uint8_t sum = check_sum((uint8_t*)parser->buf, parser->buf[2] + 4);

        // 调整标志位
        parser->FE = 0;
        parser->FH = 0;
        parser->data_len = 0;

        // 校验和为0时校验成功
        if (!sum) parser->flag = 1;
    }

    parser->data_len++;
    if (parser->data_len >= parser->buf_len) parser->data_len = 0;
}

/**
 *  函数功能：数据帧处理
 *  入口参数：数据帧首地址
 *  返 回 值：None
 */
void process_data(uint8_t *buf) {
    Function_Code code = (Function_Code)buf[3];
    switch (code) {
        case NONE: break;

        case GET_SPEED: break;

        case SET_SPEED: break;

        case GET_IMU_TEMPERATURE: {
            FloatHexUnion fh;
            fh.float_value = icm_get_temperature();
            for (int i = 0; i < 4; i++) {
                com_tx_data[4 + i] = fh.hex[3 - i];
            }

            com_tx_data[0] = 0xFE;
            com_tx_data[1] = 0xEF;
            com_tx_data[2] = 0x05;
            com_tx_data[3] = 0x03;
            com_tx_data[8] = check_sum((uint8_t*)com_tx_data, 8);

            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)com_tx_data, 9);
            break;
        }

        case GET_IMU: {
            FloatHexUnion fh;
            fh.float_value = icm_raw_data.accel_x;
            for (int i = 0; i < 4; i++) {
                com_tx_data[4 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.accel_y;
            for (int i = 0; i < 4; i++) {
                com_tx_data[8 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.accel_z;
            for (int i = 0; i < 4; i++) {
                com_tx_data[12 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_x;
            for (int i = 0; i < 4; i++) {
                com_tx_data[16 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_y;
            for (int i = 0; i < 4; i++) {
                com_tx_data[20 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_z;
            for (int i = 0; i < 4; i++) {
                com_tx_data[24 + i] = fh.hex[3 - i];
            }

            com_tx_data[0] = 0xFE;
            com_tx_data[1] = 0xEF;
            com_tx_data[2] = 0x19;
            com_tx_data[3] = 0x04;
            com_tx_data[28] = check_sum((uint8_t*)com_tx_data, 28);

            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)com_tx_data, 29);
            break;
        }
    }
}
