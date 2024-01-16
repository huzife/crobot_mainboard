#include "host_com.h"
#include "icm42605.h"
#include "swsr_queue.h"
#include "los_event.h"
#include "los_memory.h"
#include "los_task.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

EVENT_CB_S host_com_event;

typedef enum {
    HOST_COM_RX_HEADER_FE,
    HOST_COM_RX_HEADER_EF,
    HOST_COM_RX_LEN,
    HOST_COM_RX_DATA
} Host_Com_Rx_State;

static struct {
    Host_Com_Rx_State state;
    uint8_t* buf;
    uint32_t buf_len;
    uint32_t data_len;
    SWSR_Queue rx_queue;
} host_com_cb;

// 浮点数-十六进制转换
typedef union {
    float val;
    uint8_t hex[4];
} Float_Hex;

inline static void float_to_hex(float val, uint8_t* hex) {
    Float_Hex fh;
    fh.val = val;
    hex[0] = fh.hex[3];
    hex[1] = fh.hex[2];
    hex[2] = fh.hex[1];
    hex[3] = fh.hex[0];
}

inline static void hex_to_float(uint8_t* hex, float* val) {
    Float_Hex fh;
    fh.hex[0] = hex[3];
    fh.hex[1] = hex[2];
    fh.hex[2] = hex[1];
    fh.hex[3] = hex[0];
    *val = fh.val;
}

static void get_imu_temperature_func() {
    host_com_cb.buf[2] = 0x05;
    host_com_cb.buf[3] = 0x03;

    float_to_hex(icm_get_temperature(), host_com_cb.buf + 4);
}

static void get_imu_data_func() {
    host_com_cb.buf[2] = 0x19;
    host_com_cb.buf[3] = 0x04;

    icm_get_raw_data(&icm_raw_data);
    float_to_hex(icm_raw_data.accel_x, host_com_cb.buf + 4);
    float_to_hex(icm_raw_data.accel_y, host_com_cb.buf + 8);
    float_to_hex(icm_raw_data.accel_z, host_com_cb.buf + 12);
    float_to_hex(icm_raw_data.angular_x, host_com_cb.buf + 16);
    float_to_hex(icm_raw_data.angular_y, host_com_cb.buf + 20);
    float_to_hex(icm_raw_data.angular_z, host_com_cb.buf + 24);
}

void host_com_init(uint8_t* pool, uint16_t buf_len) {
    host_com_cb.state = HOST_COM_RX_HEADER_FE;
    host_com_cb.buf = (uint8_t*)LOS_MemAlloc(pool, buf_len);
    host_com_cb.buf_len = buf_len;
    host_com_cb.data_len = 0;
    swsr_queue_init(&host_com_cb.rx_queue, pool, 2 * buf_len);

    host_com_cb.buf[0] = 0xFE;
    host_com_cb.buf[1] = 0xEF;
}

bool host_com_parse() {
    // printf("current state: %d, received: %x\n", host_com_cb.state, data);
    uint8_t data;
    if (!swsr_queue_pop(&host_com_cb.rx_queue, &data))
        return false;

    switch (host_com_cb.state) {
        case HOST_COM_RX_HEADER_FE:
            if (data == 0xFE)
                host_com_cb.state = HOST_COM_RX_HEADER_EF;
            break;
        case HOST_COM_RX_HEADER_EF:
            if (data == 0xEF)
                host_com_cb.state = HOST_COM_RX_LEN;
            else if (data != 0xFE)
                host_com_cb.state = HOST_COM_RX_HEADER_FE;
            break;
        case HOST_COM_RX_LEN:
            if (data + 3 > host_com_cb.buf_len) {
                host_com_cb.state = HOST_COM_RX_HEADER_FE;
            } else {
                host_com_cb.state = HOST_COM_RX_DATA;
                host_com_cb.data_len = 0;
                host_com_cb.buf[2] = data;
            }
            break;
        case HOST_COM_RX_DATA:
            host_com_cb.buf[3 + host_com_cb.data_len] = data;
            if (++(host_com_cb.data_len) == host_com_cb.buf[2]) {
                host_com_cb.state = HOST_COM_RX_HEADER_FE;
                return true;
            }
            break;
    }

    return false;
}

void host_com_process() {
    switch ((Function_Code)host_com_cb.buf[3]) {
        case NONE: break;

        case GET_SPEED: break;

        case SET_SPEED: break;

        case GET_IMU_TEMPERATURE:
            get_imu_temperature_func();
            break;

        case GET_IMU_DATA:
            get_imu_data_func();
            break;
    }

    LOS_EventRead(&host_com_event,
                  HOST_COM_TX_DONE,
                  LOS_WAITMODE_AND | LOS_WAITMODE_CLR,
                  LOS_WAIT_FOREVER);
    CDC_Transmit_FS(host_com_cb.buf, host_com_cb.buf[2] + 3);
}

void host_rx_callback(uint8_t* buf, uint32_t len) {
    for (int i = 0; i < len; i++) {
        swsr_queue_push(&host_com_cb.rx_queue, buf[i]);
    }
}

void host_tx_callback() {
    LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE);
}
