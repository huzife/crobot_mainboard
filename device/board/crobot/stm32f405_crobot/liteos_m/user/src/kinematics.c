#include "kinematics.h"
#include "kinematics_impl.h"
#include "mem_pool.h"
#include "modbus_rtu.h"
#include "los_atomic.h"
#include "los_memory.h"
#include "los_mux.h"
#include <math.h>
#include <stdbool.h>

static Kinematics k_inverse;
static Kinematics k_forward;
static Odometry odometry;
static uint32_t target_velocity_mtx;
static uint32_t current_velocity_mtx;
static uint32_t odometry_mtx;
static volatile int velocity_avaliable;
static uint32_t last_tick;

static double linear_factor;
static double angular_factor;
static double pid_interval;
const uint32_t CPR = 1580;

inline static void kinematics_struct_init(Kinematics* k) {
    k->velocity.linear_x = 0.0;
    k->velocity.linear_y = 0.0;
    k->velocity.angular_z = 0.0;
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
    linear_factor = 1.0;
    angular_factor = 1.0;
    pid_interval = 0.05;

    kinematics_struct_init(&k_inverse);
    kinematics_struct_init(&k_forward);
    odometry.position_x = 0.0;
    odometry.position_y = 0.0;
    odometry.direction = 0.0;
}

void kinematics_get_odometry_and_velocity(Odometry* odom, Velocity* vel) {
    LOS_MuxPend(current_velocity_mtx, 50);
    LOS_MuxPend(odometry_mtx, 50);
    *vel = k_forward.velocity;
    *odom = odometry;
    LOS_MuxPost(odometry_mtx);
    LOS_MuxPost(current_velocity_mtx);
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

inline static void set_target_speed(double speeds[]) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        k_inverse.speeds[i] = speeds[i] * pid_interval * CPR / (2 * M_PI);
    }
}

inline static void get_current_speed(double speeds[]) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        speeds[i] = (double)k_forward.speeds[i] / pid_interval / CPR * (2 * M_PI);
    }
}

void kinematics_handle_velocity() {
    if (LOS_AtomicCmpXchg32bits(&velocity_avaliable, 0, 1))
        return;

    Velocity velocity;
    double speeds[WHEEL_NUM];

    LOS_MuxPend(target_velocity_mtx, 50);
    velocity = k_inverse.velocity;
    LOS_MuxPost(target_velocity_mtx);

    correct_target_velocity(&velocity);
    kinematics_inverse_func(velocity, speeds);
    set_target_speed(speeds);
    modbus_set_holding_regs(0, 1, 0, (uint16_t*)k_inverse.speeds, WHEEL_NUM);
}

void kinematics_update_info() {
    if (!modbus_get_input_regs(0, 1, 0, WHEEL_NUM, (uint16_t*)k_forward.speeds))
        return;

    Velocity velocity;
    double speeds[WHEEL_NUM];

    get_current_speed(speeds);
    kinematics_forward_func(speeds, &velocity);
    correct_current_velocity(&velocity);

    LOS_MuxPend(odometry_mtx, 50);
    Odometry odom = odometry;
    LOS_MuxPost(odometry_mtx);
    uint32_t cur_tick = LOS_TickCountGet();
    double dt = LOS_Tick2MS(cur_tick - last_tick) / 1000.0;
    last_tick = cur_tick;
    kinematics_update_odometry(&odom, velocity, dt);

    // set current velocity at the same time
    LOS_MuxPend(current_velocity_mtx, 50);
    LOS_MuxPend(odometry_mtx, 50);
    k_forward.velocity = velocity;
    odometry = odom;
    LOS_MuxPost(odometry_mtx);
    LOS_MuxPost(current_velocity_mtx);
}
