#include "bus_serial.h"
#include "bumper.h"
#include "host_com.h"
#include "icm42605.h"
#include "ps2.h"
#include "vel_mux.h"
#include "los_event.h"
#include "los_memory.h"
#include "los_task.h"
#include "usb_device.h"
#include <stdbool.h>

#define MEMORY_POOL_SIZE 2048
uint8_t mem_pool[MEMORY_POOL_SIZE];

uint32_t host_com_task_id;
uint32_t imu_task_id;
uint32_t bumper_task_id;
uint32_t controller_task_id;
uint32_t kinematics_task_id;

void host_com_task(void) {
    icm_init();
    host_com_init(mem_pool, 128);
    MX_USB_DEVICE_Init();
    LOS_EventInit(&host_com_event);
    LOS_EventWrite(&host_com_event, HOST_COM_TX_DONE);

    while (true) {
        LOS_TaskDelay(1);

        if (host_com_parse())
            host_com_process();
    }
}

void bumper_task(void) {
    int vel_id = vel_mux_register(0, 200);
    if (vel_id < 0) {
        printf("Register bumper failed\n");
        return;
    }

    Velocity_Message velocity = {0};
    velocity.id = vel_id;

    while (true) {
        velocity.linear_x = 0;
        velocity.angular_z = 0;

        Bumper_State state = bumper_check();
        if (state.left || state.front || state.right) {
            if (state.left) {
                velocity.linear_x -= 0.1;
                velocity.angular_z -= 0.4;
            }
            if (state.front) {
                velocity.linear_x -= 0.1;
            }
            if (state.right) {
                velocity.linear_x -= 0.1;
                velocity.angular_z += 0.4;
            }

            // set velocity
            for (int i = 0; i < 20; i++) {
                vel_mux_set_velocity(&velocity);
                LOS_TaskDelay(100);
            }

            // stop
            velocity.linear_x = 0.0;
            velocity.angular_z = 0.0;
            vel_mux_set_velocity(&velocity);
        }

        LOS_TaskDelay(10);
    }
}

void controller_task(void) {
    int vel_id = vel_mux_register(1, 100);
    if (vel_id < 0) {
        printf("Register controller failed\n");
        return;
    }

    Velocity_Message velocity = {0};
    velocity.id = vel_id;

    while (true) {
        ps2_read_data();
        if (ps2_state.Mode) {
            velocity.linear_x = 0.3 * ps2_state.Rocker_LY / 128;
            velocity.angular_z = -1.0 * ps2_state.Rocker_RX / 128;
            vel_mux_set_velocity(&velocity);
        }
        LOS_TaskDelay(50);
    }
}

void create_task(uint32_t* task_id,
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

void crobot_init() {
    LOS_KernelInit();

    // Initialize resources that are used by multiple tasks
    LOS_MemInit(mem_pool, MEMORY_POOL_SIZE);
    bus_serial_init(0, &huart5, NULL, 0);
    vel_mux_init(mem_pool, 8);
}

void start_tasks(void) {
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

    // unlock the task scheduling
    LOS_TaskUnlock();

    // start scheduling
    LOS_Start();
}
