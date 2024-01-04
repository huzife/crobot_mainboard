#include "host_com.h"
#include "icm42605.h"
#include "usart.h"
#include "los_memory.h"

uint8_t host_tx_buf[HOST_TX_BUF_SIZE];  // uart通信发送缓冲
uint8_t host_rx_data;
SQueue host_rx_queue;

// 浮点数-十六进制转换
typedef union {
    uint8_t hex[4];
    float float_value;
} FloatHexUnion;

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
void host_parser_init(Host_Parser *parser, uint8_t* pool, uint16_t buf_len) {
    parser->flag = 0;
    parser->FE = 0;
    parser->FH = 0;
    parser->buf_len = buf_len;
    parser->buf = (uint8_t*)LOS_MemAlloc(pool, buf_len);
    parser->data_len = 0;
}

/**
 *  函数功能：解析数据
 *  入口参数：数据解析器结构体指针、一个字节数据
 *  返 回 值：None
 */
void parse(Host_Parser *parser, uint8_t data) {
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
    switch ((Function_Code)buf[3]) {
        case NONE: break;

        case GET_SPEED: break;

        case SET_SPEED: break;

        case GET_IMU_TEMPERATURE: {
            FloatHexUnion fh;
            fh.float_value = icm_get_temperature();
            for (int i = 0; i < 4; i++) {
                host_tx_buf[4 + i] = fh.hex[3 - i];
            }

            host_tx_buf[0] = 0xFE;
            host_tx_buf[1] = 0xEF;
            host_tx_buf[2] = 0x05;
            host_tx_buf[3] = 0x03;
            host_tx_buf[8] = check_sum((uint8_t*)host_tx_buf, 8);

            break;
        }

        case GET_IMU: {
            FloatHexUnion fh;
            fh.float_value = icm_raw_data.accel_x;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[4 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.accel_y;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[8 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.accel_z;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[12 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_x;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[16 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_y;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[20 + i] = fh.hex[3 - i];
            }

            fh.float_value = icm_raw_data.angular_z;
            for (int i = 0; i < 4; i++) {
                host_tx_buf[24 + i] = fh.hex[3 - i];
            }

            host_tx_buf[0] = 0xFE;
            host_tx_buf[1] = 0xEF;
            host_tx_buf[2] = 0x19;
            host_tx_buf[3] = 0x04;
            host_tx_buf[28] = check_sum((uint8_t*)host_tx_buf, 28);

            break;
        }
    }

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)host_tx_buf, host_tx_buf[2] + 4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART1) {
    s_push(&host_rx_queue, host_rx_data);
    HAL_UART_Receive_IT(huart, &host_rx_data, 1);
  }
}
