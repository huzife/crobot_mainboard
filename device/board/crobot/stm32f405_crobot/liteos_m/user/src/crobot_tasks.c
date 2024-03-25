#include "crobot_tasks.h"
#include "bus_serial.h"
#include "bumper.h"
#include "host_com.h"
#include "icm42605.h"
#include "kinematics.h"
#include "los_atomic.h"
#include "modbus_rtu.h"
#include "ps2.h"
#include "vel_mux.h"
#include "los_event.h"
#include "los_memory.h"
#include "los_task.h"
#include "usb_device.h"
#include "tim.h"
#include <stdbool.h>

#define MEMORY_POOL_SIZE 2048
static uint8_t mem_pool[MEMORY_POOL_SIZE];

static uint32_t host_com_task_id;
static uint32_t bumper_task_id;
static uint32_t controller_task_id;
static uint32_t kinematics_task_id;

static void host_com_task() {
    icm42605_init();
    host_com_init(mem_pool, 128);
    MX_USB_DEVICE_Init();
    LOS_EventInit(&host_com_event);
    LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE);
    host_com_velocity.id = vel_mux_register(5, 100);

    while (true) {
        LOS_TaskDelay(1);

        if (host_com_parse())
            host_com_process();
    }
}

static void bumper_task() {
    int vel_id = vel_mux_register(0, 200);
    if (vel_id < 0) {
        printf("Register bumper failed\n");
        return;
    }

    Velocity_Message velocity = {vel_id, {0.0, 0.0, 0.0}};

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
    int vel_id = vel_mux_register(1, 100);
    if (vel_id < 0) {
        printf("Register controller failed\n");
        return;
    }

    Velocity_Message velocity = {vel_id, {0.0, 0.0, 0.0}};

    while (true) {
        ps2_read_data();
        if (ps2_state.Mode) {
            velocity.velocity.linear_x = 0.3 * ps2_state.Rocker_LY / 128;
            velocity.velocity.angular_z = -1.0 * ps2_state.Rocker_RX / 128;
            vel_mux_set_velocity(velocity);
        }
        LOS_TaskDelay(50);
    }
}

static void kinematics_task() {
    HAL_TIM_Base_Start_IT(&htim7);

    while (true) {
        // get interval, reset counter
        float interval = __HAL_TIM_GET_COUNTER(&htim7) / 10000.0;
        __HAL_TIM_SET_COUNTER(&htim7, 0);

        // get current speed
        uint16_t read_buf[WHEEL_NUM];
        if (modbus_rtu_get_input_regs(0, 1, 0, WHEEL_NUM, read_buf)) {
            kinematics_set_current_motor_speed(read_buf);
            kinematics_forward();
            kinematics_update_odom(interval);
        }

        // set velocity if there is any velocity message avaliable
        if (!LOS_AtomicCmpXchg32bits(&velocity_avaliable, 0, 1)) {
            kinematics_inverse();
            modbus_rtu_set_holding_regs(
                0, 1, 0, kinematics_get_target_motor_speed(), WHEEL_NUM);
        }

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
        printf("%s create failed, return code: %d\n", name, ret);
    else
        printf("%s create success\n", name);
}

static void crobot_init() {
    LOS_KernelInit();

    // Initialize resources that are used by multiple tasks
    LOS_MemInit(mem_pool, MEMORY_POOL_SIZE);
    bus_serial_init(0, &huart5, NULL, 0);
    vel_mux_init(mem_pool, 8);
}

void start_tasks() {
    crobot_init();

    // lock the task scheduling
    LOS_TaskLock();

    // create tasks
    create_task(&host_com_task_id, (TSK_ENTRY_FUNC)host_com_task,
                "host_com_task", 6, 0x1000);
    create_task(&bumper_task_id, (TSK_ENTRY_FUNC)bumper_task,
                "bumper_task", 6, 0x1000);
    // create_task(&controller_task_id, (TSK_ENTRY_FUNC)controller_task,
    //             "controller_task", 6, 0x1000);
    create_task(&kinematics_task_id, (TSK_ENTRY_FUNC)kinematics_task,
                "kinematics_task", 6, 0x1000);

    // unlock the task scheduling
    LOS_TaskUnlock();

    // start scheduling
    LOS_Start();
}
