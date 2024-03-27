#include "host_com.h"
#include "icm42605.h"
#include "kinematics.h"
#include "modbus_rtu.h"
#include "swsr_queue.h"
#include "los_event.h"
#include "los_memory.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

#define DATA_LEN host_com_cb.buf[2]
#define FUNCTION host_com_cb.buf[3]
#define DATA_START (host_com_cb.buf + 4)

EVENT_CB_S host_com_event;
Velocity_Message host_com_velocity;

typedef enum {
    HOST_COM_RX_HEADER_FE,
    HOST_COM_RX_HEADER_EF,
    HOST_COM_RX_LEN,
    HOST_COM_RX_DATA
} Host_Com_Rx_State;

// host com control block
static struct {
    Host_Com_Rx_State state;
    uint8_t* buf;
    uint32_t buf_len;
    uint32_t data_len;
    SWSR_Queue rx_queue;
} host_com_cb;

// float-hex convertion union
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

static bool set_velocity_func() {
    if (DATA_LEN != 9)
        return false;

    hex_to_float(DATA_START, &host_com_velocity.velocity.linear_x);
    hex_to_float(DATA_START + 4, &host_com_velocity.velocity.linear_y);
    hex_to_float(DATA_START + 8, &host_com_velocity.velocity.angular_z);
    vel_mux_set_velocity(host_com_velocity);
    DATA_LEN = 0;

    return true;
}

static bool get_odom_func() {
    if (DATA_LEN != 1)
        return false;

    DATA_LEN = 21;
    Velocity velocity = kinematics_get_current_velocity();
    Odometry odometry = kinematics_get_odom();
    float_to_hex(velocity.linear_x, DATA_START);
    float_to_hex(velocity.angular_z, DATA_START + 4);
    float_to_hex(odometry.position_x, DATA_START + 8);
    float_to_hex(odometry.position_y, DATA_START + 12);
    float_to_hex(odometry.direction, DATA_START + 16);

    return true;
}

static bool get_imu_temperature_func() {
    if (DATA_LEN != 1)
        return false;

    DATA_LEN = 5;
    float_to_hex(icm42605_get_temperature(), DATA_START);

    return true;
}

static bool get_imu_data_func() {
    if (DATA_LEN != 1)
        return false;

    DATA_LEN = 25;
    IMU_Data imu_data = icm42605_get_data();
    float_to_hex(imu_data.accel_x, DATA_START);
    float_to_hex(imu_data.accel_y, DATA_START + 4);
    float_to_hex(imu_data.accel_z, DATA_START + 8);
    float_to_hex(imu_data.angular_x, DATA_START + 12);
    float_to_hex(imu_data.angular_y, DATA_START + 16);
    float_to_hex(imu_data.angular_z, DATA_START + 20);

    return true;
}

static bool get_ultrasonic_range_func() {
    if (DATA_LEN != 1)
        return false;

    DATA_LEN = 3;
    uint16_t range;
    if (!modbus_rtu_get_input_regs(1, 1, 0, 1, &range))
        return false;

    *(DATA_START) = range >> 8;
    *(DATA_START + 1) = range & 0xFF;

    return true;
}

void host_com_init(uint8_t* pool, uint16_t buf_len) {
    host_com_cb.state = HOST_COM_RX_HEADER_FE;
    host_com_cb.buf = (uint8_t*)LOS_MemAlloc(pool, buf_len);
    host_com_cb.buf_len = buf_len;
    host_com_cb.data_len = 0;
    host_com_cb.buf[0] = 0xFE;
    host_com_cb.buf[1] = 0xEF;
    swsr_queue_init(&host_com_cb.rx_queue, pool, 2 * buf_len);

    MX_USB_DEVICE_Init();
}

bool host_com_parse() {
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
                DATA_LEN = data;
            }
            break;
        case HOST_COM_RX_DATA:
            host_com_cb.buf[3 + host_com_cb.data_len] = data;
            if (++(host_com_cb.data_len) == DATA_LEN) {
                host_com_cb.state = HOST_COM_RX_HEADER_FE;
                return true;
            }
            break;
    }

    return false;
}

void host_com_process() {
    bool ret;
    switch ((Function_Code)host_com_cb.buf[3]) {
        case NONE:
            ret = false;
            break;

        case SET_VELOCITY:
            ret = set_velocity_func();
            break;

        case GET_ODOM:
            ret = get_odom_func();
            break;

        case GET_IMU_TEMPERATURE:
            ret = get_imu_temperature_func();
            break;

        case GET_IMU_DATA:
            ret = get_imu_data_func();
            break;

        case GET_ULTRASONIC_RANGE:
            ret = get_ultrasonic_range_func();
            break;
    }

    if (ret && DATA_LEN) {
        LOS_EventRead(&host_com_event,
                      HOST_COM_TX_DONE,
                      LOS_WAITMODE_AND | LOS_WAITMODE_CLR,
                      500);
        CDC_Transmit_FS(host_com_cb.buf, DATA_LEN + 3);
    }
}

void host_rx_callback(uint8_t* buf, uint32_t len) {
    for (int i = 0; i < len; i++) {
        swsr_queue_push(&host_com_cb.rx_queue, buf[i]);
    }
}

void host_tx_callback() {
    LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE);
}
