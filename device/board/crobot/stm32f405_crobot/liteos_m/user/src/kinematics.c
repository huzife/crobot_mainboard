#include "kinematics.h"
#include "kinematics_impl.h"
#include "mem_pool.h"
#include "modbus_rtu.h"
#include "los_atomic.h"
#include "los_memory.h"
#include "los_mux.h"
#include <math.h>
#include <stdbool.h>

#define WHEEL_SPEED_SCALE 10000.0f
#define KINEMATICS_WINDOW_SIZE 4
const float PI = M_PI;

static Velocity velocity_accu[KINEMATICS_WINDOW_SIZE];
static Velocity velocity_sum;
static uint32_t velocity_index;
static bool is_window_full;

static Kinematics k_inverse;
static Kinematics k_forward;
static Odometry odometry;
static uint32_t target_velocity_mtx;
static uint32_t current_velocity_mtx;
static uint32_t odometry_mtx;
static uint32_t last_tick;

static volatile int velocity_avaliable;
static volatile float linear_factor;
static volatile float angular_factor;

inline static void kinematics_struct_init(Kinematics* k) {
    k->velocity.linear_x = 0.0f;
    k->velocity.linear_y = 0.0f;
    k->velocity.angular_z = 0.0f;
    k->speeds = (int16_t*)LOS_MemAlloc(mem_pool, WHEEL_NUM * 2);
    for (int i = 0; i < WHEEL_NUM; i++) {
        k->speeds[i] = 0;
    }
}

void kinematics_init() {
    LOS_MuxCreate(&target_velocity_mtx);
    LOS_MuxCreate(&current_velocity_mtx);
    LOS_MuxCreate(&odometry_mtx);
    last_tick = LOS_TickCountGet();
    velocity_avaliable = 0;
    linear_factor = 1.0f;
    angular_factor = 1.0f;

    kinematics_struct_init(&k_inverse);
    kinematics_struct_init(&k_forward);
    odometry.position_x = 0.0f;
    odometry.position_y = 0.0f;
    odometry.direction = 0.0f;

    velocity_sum.linear_x = 0.0f;
    velocity_sum.linear_y = 0.0f;
    velocity_sum.angular_z = 0.0f;
    velocity_index = 0;
    is_window_full = false;
}

void kinematics_get_odometry_and_velocity(Odometry* odom, Velocity* vel) {
    LOS_MuxPend(current_velocity_mtx, 50);
    LOS_MuxPend(odometry_mtx, 50);
    *vel = k_forward.velocity;
    *odom = odometry;
    LOS_MuxPost(odometry_mtx);
    LOS_MuxPost(current_velocity_mtx);
}

void kinematics_reset_odometry() {
    LOS_MuxPend(odometry_mtx, 50);
    odometry.direction = 0.0f;
    odometry.position_x = 0.0f;
    odometry.position_y = 0.0f;
    LOS_MuxPost(odometry_mtx);
}

void kinematics_set_correction_factor(float linear, float angular) {
    linear_factor = linear;
    angular_factor = angular;
}

void kinematics_set_target_velocity(Velocity velocity) {
    LOS_MuxPend(target_velocity_mtx, 50);
    k_inverse.velocity = velocity;
    LOS_MuxPost(target_velocity_mtx);
    LOS_AtomicSet(&velocity_avaliable, 1);
}

inline static void correct_target_velocity(Velocity* velocity) {
    velocity->linear_x /= linear_factor;
    velocity->linear_y /= linear_factor;
    velocity->angular_z /= angular_factor;
}

inline static void correct_current_velocity(Velocity* velocity) {
    velocity->linear_x *= linear_factor;
    velocity->linear_y *= linear_factor;
    velocity->angular_z *= angular_factor;
}

inline static void set_target_speed(float speeds[]) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        k_inverse.speeds[i] = speeds[i] * WHEEL_SPEED_SCALE;
    }
}

inline static void get_current_speed(float speeds[]) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        speeds[i] = k_forward.speeds[i] / WHEEL_SPEED_SCALE;
    }
}

void kinematics_handle_velocity() {
    if (LOS_AtomicCmpXchg32bits(&velocity_avaliable, 0, 1))
        return;

    Velocity velocity;
    float speeds[WHEEL_NUM];

    LOS_MuxPend(target_velocity_mtx, 50);
    velocity = k_inverse.velocity;
    LOS_MuxPost(target_velocity_mtx);

    correct_target_velocity(&velocity);
    kinematics_inverse_func(velocity, speeds);
    set_target_speed(speeds);
    modbus_set_holding_regs(0, 1, 0, (uint16_t*)k_inverse.speeds, WHEEL_NUM);
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
    if (!modbus_get_input_regs(0, 1, 0, WHEEL_NUM, (uint16_t*)k_forward.speeds))
        return;

    Velocity velocity;
    float speeds[WHEEL_NUM];

    get_current_speed(speeds);
    kinematics_forward_func(speeds, &velocity);
    correct_current_velocity(&velocity);

    LOS_MuxPend(odometry_mtx, 50);
    Odometry odom = odometry;
    LOS_MuxPost(odometry_mtx);
    uint32_t cur_tick = LOS_TickCountGet();
    float dt = LOS_Tick2MS(cur_tick - last_tick) / 1000.0f;
    last_tick = cur_tick;
    kinematics_update_odometry(&odom, velocity, dt);
    kinematics_rolling_mean(&velocity);

    // set current velocity at the same time
    LOS_MuxPend(current_velocity_mtx, 50);
    LOS_MuxPend(odometry_mtx, 50);
    k_forward.velocity = velocity;
    odometry = odom;
    LOS_MuxPost(odometry_mtx);
    LOS_MuxPost(current_velocity_mtx);
}
