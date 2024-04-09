#include "battery_voltage.h"
#include "bumper.h"
#include "host_com.h"
#include "icm42605.h"
#include "kinematics.h"
#include "mem_pool.h"
#include "modbus_rtu.h"
#include "ps2.h"
#include "ultrasonic.h"
#include "vel_mux.h"
#include "los_atomic.h"
#include "los_debug.h"
#include "los_memory.h"
#include "los_task.h"

uint32_t host_com_task_id;
uint32_t bumper_task_id;
uint32_t controller_task_id;
uint32_t kinematics_task_id;
uint32_t ultrasonic_task_id;
uint32_t imu_task_id;
uint32_t battery_voltage_task_id;

static void* host_com_task(uint32_t arg) {
    if (!host_com_init(128)) {
        PRINT_ERR("Host com init failed\n");
        return NULL;
    }

    while (true) {
        if (!host_com_parse())
            LOS_TaskDelay(1);
    }
}

static void* bumper_task(uint32_t arg) {
    int vel_id = vel_mux_register(0, 200);
    if (vel_id < 0) {
        PRINT_ERR("Register bumper velocity failed\n");
        return NULL;
    }
    Velocity_Message velocity = {vel_id, {0.0, 0.0, 0.0}};
    bumper_init();

    while (true) {
        Bumper_State state = bumper_check();
        if (state.left || state.front || state.right) {
            if (state.left) {
                velocity.velocity.linear_x -= 0.1;
                velocity.velocity.angular_z -= 0.4;
            }
            if (state.front) {
                velocity.velocity.linear_x -= 0.1;
            }
            if (state.right) {
                velocity.velocity.linear_x -= 0.1;
                velocity.velocity.angular_z += 0.4;
            }

            // set velocity
            for (int i = 0; i < 20; i++) {
                vel_mux_set_velocity(velocity);
                LOS_TaskDelay(100);
            }

            // stop
            velocity.velocity.linear_x = 0.0;
            velocity.velocity.angular_z = 0.0;
            vel_mux_set_velocity(velocity);
        }

        LOS_TaskDelay(10);
    }
}

static void* controller_task(uint32_t arg) {
    ps2_init();
    int vel_id = vel_mux_register(1, 100);
    if (vel_id < 0) {
        PRINT_ERR("Register controller failed\n");
        return NULL;
    }

    Velocity_Message velocity = {vel_id, {0.0, 0.0, 0.0}};

    while (true) {
        ps2_read_data();
        if (ps2_state.Mode) {
            velocity.velocity.linear_x = 0.3 * ps2_state.Rocker_LY / 128;
            velocity.velocity.angular_z = -1.0 * ps2_state.Rocker_RX / 128;
            vel_mux_set_velocity(velocity);
            LOS_TaskDelay(50);
        } else {
            LOS_TaskDelay(1000);
        }
    }
}

static void* kinematics_task(uint32_t arg) {
    uint16_t proto_rev;
    if (!modbus_get_input_regs(0, 1, 0xFF, 1, &proto_rev)) {
        PRINT_ERR("Failed to read protocol revision\n");
        return NULL;
    } else if (proto_rev < 3) {
        PRINT_ERR("Protocol revision too low\n");
        return NULL;
    }

    while (true) {
        // get current speed
        uint16_t read_buf[WHEEL_NUM];
        if (modbus_get_input_regs(0, 1, 0, WHEEL_NUM, read_buf)) {
            kinematics_set_current_motor_speed((int16_t*)read_buf);
            kinematics_forward();
            kinematics_update_odom();
        }

        // set velocity if there is any velocity message avaliable
        if (!LOS_AtomicCmpXchg32bits(&velocity_avaliable, 0, 1)) {
            kinematics_inverse();
            modbus_set_holding_regs(
                0, 1, 0, (uint16_t*)kinematics_get_target_motor_speed(), WHEEL_NUM);
        }

        LOS_TaskDelay(50);
    }
}

static void* ultrasonic_task(uint32_t arg) {
    ultrasonic_init();
    while (true) {
        ultrasonic_update_range();
        LOS_TaskDelay(50);
    }
}

static void* imu_task(uint32_t arg) {
    icm42605_init();
    int cnt = 0;
    while (true) {
        // imu data update rate: 100Hz
        icm42605_update_data();

        // imu temperature update rate: 50Hz
        if (++cnt == 50) {
            icm42605_update_temperature();
            cnt = 0;
        }

        LOS_TaskDelay(10);
    }
}

static void* battery_voltage_task(uint32_t arg) {
    battery_voltage_init();
    while (true) {
        battery_voltage_update();
        LOS_TaskDelay(1000);
    }
}

static void create_task(uint32_t* task_id,
                        TSK_ENTRY_FUNC entry_func,
                        char* name,
                        uint16_t priority,
                        uint32_t stack_size) {
    TSK_INIT_PARAM_S init_param = {0};
    init_param.pfnTaskEntry = entry_func;
    init_param.pcName = name;
    init_param.usTaskPrio = priority;
    init_param.uwStackSize = stack_size;

    uint32_t ret = LOS_TaskCreate(task_id, &init_param);
    if (ret != LOS_OK)
        PRINTK("%s create failed, return code: %d\n", name, ret);
    else
        PRINTK("%s create success\n", name);
}

static void crobot_init() {
    LOS_KernelInit();

    // Initialize resources that are used by multiple tasks
    LOS_MemInit(mem_pool, MEMORY_POOL_SIZE);
    modbus_init();
    kinematics_init();
    vel_mux_init(8);
}

void crobot_start() {
    crobot_init();

    LOS_TaskLock();
    create_task(&host_com_task_id, host_com_task, "host_com_task", 6, 0x1000);
    create_task(&bumper_task_id, bumper_task, "bumper_task", 6, 0x1000);
    create_task(&controller_task_id, controller_task, "controller_task", 6, 0x1000);
    create_task(&kinematics_task_id, kinematics_task, "kinematics_task", 6, 0x1000);
    create_task(&ultrasonic_task_id, ultrasonic_task, "ultrasonic_task", 6, 0x1000);
    create_task(&imu_task_id, imu_task, "imu_task", 6, 0x1000);
    create_task(&battery_voltage_task_id, battery_voltage_task, "battery_voltage_task", 6, 0x1000);
    LOS_TaskUnlock();

    // start scheduling
    LOS_Start();
}
