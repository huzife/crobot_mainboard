#include "host_com.h"
#include "battery_voltage.h"
#include "icm42605.h"
#include "kinematics.h"
#include "mem_pool.h"
#include "modbus_rtu.h"
#include "swsr_queue.h"
#include "vel_mux.h"
#include "ultrasonic.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "los_debug.h"
#include "los_event.h"
#include "los_memory.h"
#include <stdbool.h>

#define HOST_COM_VEL_PRIORITY 5
#define HOST_COM_VEL_EXPIRY_TIME 100

#define DATA_LEN host_com_cb.buf[2]
#define MESSAGE_TYPE host_com_cb.buf[3]
#define DATA_START (host_com_cb.buf + 4)

static EVENT_CB_S host_com_event;
static Velocity_Message host_com_velocity;

typedef enum {
    SET_PID_INTERVAL,
    SET_COUNT_PER_REV,
    SET_ROBOT_BASE,
    SET_CORRECTION_FACTOR,
    SET_VELOCITY,
    RESET_ODOMETRY,
    GET_ODOMETRY,
    GET_IMU_TEMPERATURE,
    GET_IMU_DATA,
    GET_ULTRASONIC_RANGE,
    GET_BATTERY_VOLTAGE,
    MESSAGE_TYPE_MAX
} Message_Type;

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

static bool set_pid_interval_func() {
    if (DATA_LEN != 3)
        return false;

    uint16_t val = ((*DATA_START) << 8) | *(DATA_START + 1);
    modbus_set_holding_reg(0, 1, 0x13, val);
    DATA_LEN = 0;

    return true;
}

static bool set_count_per_rev_func() {
    if (DATA_LEN != 3)
        return false;

    uint16_t val = ((*DATA_START) << 8) | *(DATA_START + 1);
    modbus_set_holding_reg(0, 1, 0x14, val);
    DATA_LEN = 0;

    return true;
}

static bool set_robot_base_func() {
    Kinematics_Robot_Base type = *(DATA_START);
    bool ret = kinematics_set_robot_base(type, DATA_START + 1, DATA_LEN - 2);
    DATA_LEN = 0;

    return ret;
}

static bool set_correction_factor_func() {
    if (DATA_LEN != 9)
        return false;

    float linear;
    float angular;
    hex_to_float(DATA_START, &linear);
    hex_to_float(DATA_START + 4, &angular);
    kinematics_set_correction_factor(linear, angular);
    DATA_LEN = 0;

    return true;
}

static bool set_velocity_func() {
    if (DATA_LEN != 13)
        return false;

    hex_to_float(DATA_START, &host_com_velocity.velocity.linear_x);
    hex_to_float(DATA_START + 4, &host_com_velocity.velocity.linear_y);
    hex_to_float(DATA_START + 8, &host_com_velocity.velocity.angular_z);
    vel_mux_set_velocity(host_com_velocity);
    DATA_LEN = 0;

    return true;
}

static bool reset_odometry_func() {
    if (DATA_LEN != 1)
        return false;

    kinematics_reset_odometry();
    DATA_LEN = 0;

    return true;
}

static bool get_odometry_func() {
    if (DATA_LEN != 1)
        return false;

    Velocity velocity;
    Odometry odometry;
    kinematics_get_odometry_and_velocity(&odometry, &velocity);
    float_to_hex(velocity.linear_x, DATA_START);
    float_to_hex(velocity.linear_y, DATA_START + 4);
    float_to_hex(velocity.angular_z, DATA_START + 8);
    float_to_hex(odometry.position_x, DATA_START + 12);
    float_to_hex(odometry.position_y, DATA_START + 16);
    float_to_hex(odometry.direction, DATA_START + 20);
    DATA_LEN = 25;

    return true;
}

static bool get_imu_temperature_func() {
    if (DATA_LEN != 1)
        return false;

    float_to_hex(icm42605_get_temperature(), DATA_START);
    DATA_LEN = 5;

    return true;
}

static bool get_imu_data_func() {
    if (DATA_LEN != 1)
        return false;

    IMU_Data imu_data = icm42605_get_data();
    float_to_hex(imu_data.accel_x, DATA_START);
    float_to_hex(imu_data.accel_y, DATA_START + 4);
    float_to_hex(imu_data.accel_z, DATA_START + 8);
    float_to_hex(imu_data.angular_x, DATA_START + 12);
    float_to_hex(imu_data.angular_y, DATA_START + 16);
    float_to_hex(imu_data.angular_z, DATA_START + 20);
    DATA_LEN = 25;

    return true;
}

static bool get_ultrasonic_range_func() {
    if (DATA_LEN != 1)
        return false;

    uint16_t range = ultrasonic_get_range();
    *(DATA_START) = range >> 8;
    *(DATA_START + 1) = range & 0xFF;
    DATA_LEN = 3;

    return true;
}

static bool get_battery_voltage_func() {
    if (DATA_LEN != 1)
        return false;

    float_to_hex(battery_get_voltage(), DATA_START);
    DATA_LEN = 5;

    return true;
}

static void host_com_process() {
    if (MESSAGE_TYPE >= MESSAGE_TYPE_MAX) {
        PRINT_ERR("Invalid function code: %d\n", MESSAGE_TYPE);
        return;
    }

    bool ret;
    switch ((Message_Type)MESSAGE_TYPE) {
        case SET_PID_INTERVAL:
            ret = set_pid_interval_func();
            break;

        case SET_COUNT_PER_REV:
            ret = set_count_per_rev_func();
            break;

        case SET_ROBOT_BASE:
            ret = set_robot_base_func();
            break;

        case SET_CORRECTION_FACTOR:
            ret = set_correction_factor_func();
            break;

        case SET_VELOCITY:
            ret = set_velocity_func();
            break;

        case RESET_ODOMETRY:
            ret = reset_odometry_func();
            break;

        case GET_ODOMETRY:
            ret = get_odometry_func();
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

        case GET_BATTERY_VOLTAGE:
            ret = get_battery_voltage_func();
            break;

        case MESSAGE_TYPE_MAX: // unreachable branch
            ret = false;
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

bool host_com_init(uint16_t buf_len) {
    host_com_cb.state = HOST_COM_RX_HEADER_FE;
    host_com_cb.buf = (uint8_t*)LOS_MemAlloc(mem_pool, buf_len);
    host_com_cb.buf_len = buf_len;
    host_com_cb.data_len = 0;
    host_com_cb.buf[0] = 0xFE;
    host_com_cb.buf[1] = 0xEF;
    swsr_queue_init(&host_com_cb.rx_queue, mem_pool, 2 * buf_len);

    MX_USB_DEVICE_Init();

    if (LOS_EventInit(&host_com_event) != LOS_OK) {
        PRINT_ERR("Failed to init host_com_event\n");
        return false;
    }

    if (LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE) != LOS_OK) {
        PRINT_ERR("Failed to write HOST_COM_TX_DONE to host_com_event\n");
        return false;
    }

    int vel_id = vel_mux_register(HOST_COM_VEL_PRIORITY, HOST_COM_VEL_EXPIRY_TIME);
    if (vel_id < 0) {
        PRINT_ERR("Register host com velocity failed\n");
        return false;
    }
    host_com_velocity.id = vel_id;

    return true;
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
                host_com_process();
            }
            break;
    }

    return true;
}

void host_rx_callback(uint8_t* buf, uint32_t len) {
    for (int i = 0; i < len; i++) {
        swsr_queue_push(&host_com_cb.rx_queue, buf[i]);
    }
}

void host_tx_callback() {
    LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE);
}
