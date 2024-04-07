#include "crobot.h"
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

static void host_com_task() {
    if (!host_com_init(128)) {
        PRINT_ERR("Host com init failed\n");
        return;
    }

    while (true) {
        if (!host_com_parse())
            LOS_TaskDelay(1);
    }
}

static void bumper_task() {
    int vel_id = vel_mux_register(0, 200);
    if (vel_id < 0) {
        PRINT_ERR("Register bumper velocity failed\n");
        return;
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

static void controller_task() {
    ps2_init();
    int vel_id = vel_mux_register(1, 100);
    if (vel_id < 0) {
        PRINT_ERR("Register controller failed\n");
        return;
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

static void kinematics_task() {
    uint16_t proto_rev;
    if (!modbus_get_input_regs(0, 1, 0xFF, 1, &proto_rev)) {
        PRINT_ERR("Failed to read protocol revision\n");
        return;
    } else if (proto_rev < 3) {
        PRINT_ERR("Protocol revision too low\n");
        return;
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

static void ultrasonic_task() {
    while (true) {
        uint16_t range;
        modbus_get_input_regs(1, 1, 0, 1, &range);
        LOS_AtomicSet(&ultrasonic_range, (int)range);
        LOS_TaskDelay(50);
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
    icm42605_init();
    modbus_init();
    vel_mux_init(8);
}

void crobot_start() {
    crobot_init();

    // lock the task scheduling
    LOS_TaskLock();

    // create tasks
    create_task(&host_com_task_id, (TSK_ENTRY_FUNC)host_com_task,
                "host_com_task", 6, 0x1000);
    create_task(&bumper_task_id, (TSK_ENTRY_FUNC)bumper_task,
                "bumper_task", 6, 0x1000);
    create_task(&controller_task_id, (TSK_ENTRY_FUNC)controller_task,
                "controller_task", 6, 0x1000);
    create_task(&kinematics_task_id, (TSK_ENTRY_FUNC)kinematics_task,
                "kinematics_task", 6, 0x1000);
    create_task(&ultrasonic_task_id, (TSK_ENTRY_FUNC)ultrasonic_task,
                "ultrasonic_task", 6, 0x1000);

    // unlock the task scheduling
    LOS_TaskUnlock();

    // start scheduling
    LOS_Start();
}
