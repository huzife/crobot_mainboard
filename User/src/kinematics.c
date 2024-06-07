#include "kinematics.h"
#include "kinematics_impl/kinematics_2wd.h"
#include "kinematics_impl/kinematics_3wo.h"
#include "kinematics_impl/kinematics_4wd.h"
#include "kinematics_impl/kinematics_4mec.h"
#include "crobot_atomic.h"
#include "crobot_debug.h"
#include "modbus_rtu.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

#define CCONFIG_RESOLUTION 0.1f
#define WHEEL_SPEED_SCALE 1000.0f
#define KINEMATICS_WINDOW_SIZE 4
const float PI = M_PI;

typedef enum  {
    KINEMATICS_INVALID,
    KINEMATICS_READY,
    KINEMATICS_RUNNING
} Kinematics_State;

static struct {
    Kinematics inverse;
    Kinematics forward;
    Odometry odometry;
    SemaphoreHandle_t target_velocity_mtx;
    SemaphoreHandle_t current_velocity_mtx;
    SemaphoreHandle_t odometry_mtx;
    TickType_t last_tick;
    volatile int velocity_avaliable;
    volatile float linear_x_factor;
    volatile float linear_y_factor;
    volatile float angular_factor;
    Kinematics_State state;
    uint32_t wheel_num;
    void (*update_odometry_func)(Odometry* odometry, Velocity velocity, float dt);
    void (*inverse_func)(Velocity velocity, float speeds[]);
    void (*forward_func)(float speeds[], Velocity* velocity);
} kinematics;

static Velocity velocity_accu[KINEMATICS_WINDOW_SIZE];
static Velocity velocity_sum;
static uint32_t velocity_index;
static bool is_window_full;

const uint32_t PARAM_SIZE[] = {
    sizeof(Kinematics_2WD_Param),
    sizeof(Kinematics_3WO_Param),
    sizeof(Kinematics_4WD_Param),
    sizeof(Kinematics_4MEC_Param)
};

inline static void kinematics_alloc_speeds() {
    if (kinematics.inverse.speeds)
        vPortFree(kinematics.inverse.speeds);
    if (kinematics.forward.speeds)
        vPortFree(kinematics.forward.speeds);

    if (kinematics.wheel_num) {
        kinematics.inverse.speeds = pvPortMalloc(kinematics.wheel_num * 2);
        kinematics.forward.speeds = pvPortMalloc(kinematics.wheel_num * 2);
        for (int i = 0; i < kinematics.wheel_num; i++) {
            kinematics.inverse.speeds[i] = 0;
            kinematics.forward.speeds[i] = 0;
        }
    }
}

static void kinematics_robot_base_init() {
#ifdef CROBOT_BASE_NONE
    kinematics.state = KINEMATICS_INVALID;
    kinematics.wheel_num = 0;
    kinematics.update_odometry_func = NULL;
    kinematics.inverse_func = NULL;
    kinematics.forward_func = NULL;
#else
#if defined CROBOT_BASE_2WD
    kinematics.wheel_num = 2;
    kinematics.update_odometry_func = kinematics_update_odometry_2wd;
    kinematics.inverse_func = kinematics_inverse_2wd;
    kinematics.forward_func = kinematics_forward_2wd;
    Kinematics_2WD_Param param;
    param.radius = CROBOT_BASE_2WD_RADIUS * CCONFIG_RESOLUTION;
    param.separation = CROBOT_BASE_2WD_SEPARATION * CCONFIG_RESOLUTION;
    kinematics_set_param_2wd(param);
#elif defined CROBOT_BASE_3WO
    kinematics.wheel_num = 3;
    kinematics.update_odometry_func = kinematics_update_odometry_3wo;
    kinematics.inverse_func = kinematics_inverse_3wo;
    kinematics.forward_func = kinematics_forward_3wo;
    Kinematics_3WO_Param param;
    param.radius = CROBOT_BASE_3WO_RADIUS * CCONFIG_RESOLUTION;
    param.distance = CROBOT_BASE_3WO_DISTANCE * CCONFIG_RESOLUTION;
    kinematics_set_param_3wo(param);
#elif defined CROBOT_BASE_4WD
    kinematics.wheel_num = 4;
    kinematics.update_odometry_func = kinematics_update_odometry_4wd;
    kinematics.inverse_func = kinematics_inverse_4wd;
    kinematics.forward_func = kinematics_forward_4wd;
    Kinematics_4WD_Param param;
    param.radius = CROBOT_BASE_4WD_RADIUS * CCONFIG_RESOLUTION;
    param.separation = CROBOT_BASE_4WD_SEPARATION * CCONFIG_RESOLUTION;
    kinematics_set_param_4wd(param);
#elif defined CROBOT_BASE_4MEC
    kinematics.wheel_num = 4;
    kinematics.update_odometry_func = kinematics_update_odometry_4mec;
    kinematics.inverse_func = kinematics_inverse_4mec;
    kinematics.forward_func = kinematics_forward_4mec;
    Kinematics_4MEC_Param param;
    param.radius = CROBOT_BASE_4MEC_RADIUS * CCONFIG_RESOLUTION;
    param.distance_x = CROBOT_BASE_4MEC_DISTANCE_X * CCONFIG_RESOLUTION;
    param.distance_y = CROBOT_BASE_4MEC_DISTANCE_Y * CCONFIG_RESOLUTION;
    kinematics_set_param_4mec(param);
#endif
    kinematics.state = KINEMATICS_READY;
#endif
}

void kinematics_init() {
    // kinematics params init
    kinematics.odometry.position_x = 0.0f;
    kinematics.odometry.position_y = 0.0f;
    kinematics.odometry.direction = 0.0f;
    kinematics.target_velocity_mtx = xSemaphoreCreateMutex();
    kinematics.current_velocity_mtx = xSemaphoreCreateMutex();
    kinematics.odometry_mtx = xSemaphoreCreateMutex();
    kinematics.last_tick = xTaskGetTickCount();
    kinematics.velocity_avaliable = 0;
    kinematics.linear_x_factor = 1.0f;
    kinematics.linear_y_factor = 1.0f;
    kinematics.angular_factor = 1.0f;
    kinematics_robot_base_init();
    kinematics.inverse.speeds = NULL;
    kinematics.forward.speeds = NULL;
    kinematics_alloc_speeds();

    // rolling mean init
    velocity_sum.linear_x = 0.0f;
    velocity_sum.linear_y = 0.0f;
    velocity_sum.angular_z = 0.0f;
    velocity_index = 0;
    is_window_full = false;
}

bool kinematics_set_robot_base(Kinematics_Robot_Base type, void* params, uint32_t size) {
    if (size != PARAM_SIZE[type]) {
        PRINT_ERR("size: %d, type: %d, paramsize: %d\n", size, type, PARAM_SIZE[type]);
        return false;
    }

    // If it's not in invalid state, wait until it's ready and set state to invalid
    if (atomic_read((const volatile int*)&kinematics.state)) {
        while (atomic_cmp_and_set((volatile int*)&kinematics.state,
                                  KINEMATICS_INVALID,
                                  KINEMATICS_READY)) {}
    }

    switch (type) {
        case KINEMATICS_ROBOT_BASE_2WD:
            kinematics.wheel_num = 2;
            kinematics.update_odometry_func = kinematics_update_odometry_2wd;
            kinematics.inverse_func = kinematics_inverse_2wd;
            kinematics.forward_func = kinematics_forward_2wd;
            Kinematics_2WD_Param param_2wd;
            memcpy(&param_2wd, params, size);
            kinematics_set_param_2wd(param_2wd);
            break;
        case KINEMATICS_ROBOT_BASE_3WO:
            kinematics.wheel_num = 3;
            kinematics.update_odometry_func = kinematics_update_odometry_3wo;
            kinematics.inverse_func = kinematics_inverse_3wo;
            kinematics.forward_func = kinematics_forward_3wo;
            Kinematics_3WO_Param param_3wo;
            memcpy(&param_3wo, params, size);
            kinematics_set_param_3wo(param_3wo);
            break;
        case KINEMATICS_ROBOT_BASE_4WD:
            kinematics.wheel_num = 4;
            kinematics.update_odometry_func = kinematics_update_odometry_4wd;
            kinematics.inverse_func = kinematics_inverse_4wd;
            kinematics.forward_func = kinematics_forward_4wd;
            Kinematics_4WD_Param param_4wd;
            memcpy(&param_4wd, params, size);
            kinematics_set_param_4wd(param_4wd);
            break;
        case KINEMATICS_ROBOT_BASE_4MEC:
            kinematics.wheel_num = 4;
            kinematics.update_odometry_func = kinematics_update_odometry_4mec;
            kinematics.inverse_func = kinematics_inverse_4mec;
            kinematics.forward_func = kinematics_forward_4mec;
            Kinematics_4MEC_Param param_4mec;
            memcpy(&param_4mec, params, size);
            kinematics_set_param_4mec(param_4mec);
            break;
    }
    kinematics_alloc_speeds();
    atomic_set((volatile int*)&kinematics.state, KINEMATICS_READY);

    return true;
}

void kinematics_get_odometry_and_velocity(Odometry* odom, Velocity* vel) {
    xSemaphoreTake(kinematics.current_velocity_mtx, 50);
    xSemaphoreTake(kinematics.odometry_mtx, 50);
    *vel = kinematics.forward.velocity;
    *odom = kinematics.odometry;
    xSemaphoreGive(kinematics.odometry_mtx);
    xSemaphoreGive(kinematics.current_velocity_mtx);
}

void kinematics_reset_odometry() {
    xSemaphoreTake(kinematics.odometry_mtx, 50);
    kinematics.odometry.direction = 0.0f;
    kinematics.odometry.position_x = 0.0f;
    kinematics.odometry.position_y = 0.0f;
    xSemaphoreGive(kinematics.odometry_mtx);
}

void kinematics_set_correction_factor(float linear_x, float linear_y, float angular) {
    kinematics.linear_x_factor = linear_x;
    kinematics.linear_y_factor = linear_y;
    kinematics.angular_factor = angular;
}

void kinematics_set_target_velocity(Velocity velocity) {
    xSemaphoreTake(kinematics.target_velocity_mtx, 50);
    kinematics.inverse.velocity = velocity;
    xSemaphoreGive(kinematics.target_velocity_mtx);
    atomic_set(&kinematics.velocity_avaliable, 1);
}

inline static void correct_target_velocity(Velocity* velocity) {
    velocity->linear_x /= kinematics.linear_x_factor;
    velocity->linear_y /= kinematics.linear_y_factor;
    velocity->angular_z /= kinematics.angular_factor;
}

inline static void correct_current_velocity(Velocity* velocity) {
    velocity->linear_x *= kinematics.linear_x_factor;
    velocity->linear_y *= kinematics.linear_y_factor;
    velocity->angular_z *= kinematics.angular_factor;
}

inline static void set_target_speed(float speeds[]) {
    for (int i = 0; i < kinematics.wheel_num; i++) {
        kinematics.inverse.speeds[i] = speeds[i] * WHEEL_SPEED_SCALE;
    }
}

inline static void get_current_speed(float speeds[]) {
    for (int i = 0; i < kinematics.wheel_num; i++) {
        speeds[i] = kinematics.forward.speeds[i] / WHEEL_SPEED_SCALE;
    }
}

void kinematics_handle_velocity() {
    if (atomic_cmp_and_set(&kinematics.velocity_avaliable, 0, 1))
        return;

    kinematics.velocity_avaliable = 0;

    Velocity velocity;
    float speeds[kinematics.wheel_num];

    xSemaphoreTake(kinematics.target_velocity_mtx, 50);
    velocity = kinematics.inverse.velocity;
    xSemaphoreGive(kinematics.target_velocity_mtx);

    correct_target_velocity(&velocity);
    kinematics.inverse_func(velocity, speeds);
    set_target_speed(speeds);
    modbus_set_holding_regs(0, 1, 0, (uint16_t*)kinematics.inverse.speeds, kinematics.wheel_num);
}

static void kinematics_rolling_mean(Velocity* velocity) {
    if (is_window_full) {
        velocity_sum.linear_x -= velocity_accu[velocity_index].linear_x;
        velocity_sum.linear_y -= velocity_accu[velocity_index].linear_y;
        velocity_sum.angular_z -= velocity_accu[velocity_index].angular_z;
    }

    velocity_sum.linear_x += velocity->linear_x;
    velocity_sum.linear_y += velocity->linear_y;
    velocity_sum.angular_z += velocity->angular_z;
    velocity_accu[velocity_index] = *velocity;

    if (++velocity_index == KINEMATICS_WINDOW_SIZE) {
        velocity_index = 0;
        if (!is_window_full)
            is_window_full = true;
    }

    // calculate the mean of velocity
    uint32_t size = is_window_full ? KINEMATICS_WINDOW_SIZE
                                   : velocity_index;
    velocity->linear_x = velocity_sum.linear_x / size;
    velocity->linear_y = velocity_sum.linear_y / size;
    velocity->angular_z = velocity_sum.angular_z / size;
    if (fabsf(velocity->linear_x) < 1e-6f)
        velocity->linear_x = 0.0f;
    if (fabsf(velocity->linear_y) < 1e-6f)
        velocity->linear_y = 0.0f;
    if (fabsf(velocity->angular_z) < 1e-6f)
        velocity->angular_z = 0.0f;
}

void kinematics_update_info() {
    if (!modbus_get_input_regs(0, 1, 0, kinematics.wheel_num, (uint16_t*)kinematics.forward.speeds))
        return;

    Velocity velocity;
    float speeds[kinematics.wheel_num];

    get_current_speed(speeds);
    kinematics.forward_func(speeds, &velocity);
    correct_current_velocity(&velocity);

    xSemaphoreTake(kinematics.odometry_mtx, 50);
    Odometry odom = kinematics.odometry;
    xSemaphoreGive(kinematics.odometry_mtx);
    TickType_t cur_tick = xTaskGetTickCount();
    float dt = (float)(cur_tick - kinematics.last_tick) / configTICK_RATE_HZ;
    kinematics.last_tick = cur_tick;
    kinematics.update_odometry_func(&odom, velocity, dt);
    kinematics_rolling_mean(&velocity);

    // set current velocity at the same time
    xSemaphoreTake(kinematics.current_velocity_mtx, 50);
    xSemaphoreTake(kinematics.odometry_mtx, 50);
    kinematics.forward.velocity = velocity;
    kinematics.odometry = odom;
    xSemaphoreGive(kinematics.odometry_mtx);
    xSemaphoreGive(kinematics.current_velocity_mtx);
}

bool kinematics_start() {
    return !atomic_cmp_and_set((volatile int*)&kinematics.state,
                                KINEMATICS_RUNNING,
                                KINEMATICS_READY);
}

void kinematics_end() {
    atomic_set((volatile int*)&kinematics.state, KINEMATICS_READY);
}
