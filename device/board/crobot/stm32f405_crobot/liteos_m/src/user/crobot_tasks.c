#include "bus_serial.h"
#include "host_com.h"
#include "icm42605.h"
#include "kinematics.h"
#include "modbus_rtu.h"
#include "ps2.h"
#include "los_memory.h"
#include "los_queue.h"
#include "los_task.h"
#include "los_tick.h"
#include "los_interrupt.h"
#include "usart.h"
#include <stdbool.h>

#define MEMORY_POOL_SIZE 2048
uint8_t mem_pool[MEMORY_POOL_SIZE];

void communication_task(void) {
    // printf("communication_task\n");
    Host_Parser parser;
    squeue_init(&host_rx_queue, mem_pool, 128);
    host_parser_init(&parser, mem_pool, 128);
    HAL_UART_Receive_IT(&huart1, &host_rx_data, 1);

    while (true) {
        LOS_TaskDelay(1);

        // 接收并解析数据帧
        if (!parser.flag) {
            uint8_t data;
            if (s_pop(&host_rx_queue, &data))
                parse(&parser, data);
        }

        // 处理数据
        if (parser.flag && huart1.gState == HAL_UART_STATE_READY) {
            parser.flag = 0;
            process_data((uint8_t*)parser.buf);
        }
    }
}

void imu_task(void) {
    // printf("imu_task\n");
    icm_init();
    while (true) {
        icm_get_raw_data(&icm_raw_data);
        LOS_TaskDelay(10);
    }
}

void bumper_task(void) {
    // printf("bumper_task\n");
    while (true) {
        uint8_t cnt = 0;
        if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
            ++cnt;
        if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
            ++cnt;
        if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
            ++cnt;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, !(cnt % 2));
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, cnt < 2);
        LOS_TaskDelay(10);
    }
}

void controller_task(void) {
    // kinematics_2WD_init(&kinematics[0]);
    // kinematics_2WD_init(&kinematics[1]);
    // kinematics_2WD_param_init(&kinematics_param);
    while (true) {
        ps2_read_data();
        if (ps2_state.Mode) {
            kinematics[0].linear_x = 0.3 * ps2_state.Rocker_LY / 128;
            kinematics[0].angular_z = -1.0 * ps2_state.Rocker_RX / 128;
            // kinematics_2WD_inverse(&kinematics[0], &kinematics_param);
            // printf("%.3f, %.3f, ", kinematics[0].linear_x, kinematics[0].angular_z);
            // printf("%d, %d\n", kinematics[0].speed_left, kinematics[0].speed_right);
        }
        LOS_TaskDelay(50);
    }
}

void kinematics_task(void) {
    // printf("kinematics task\n");
    kinematics_2WD_init(&kinematics[0]);
    kinematics_2WD_init(&kinematics[1]);
    kinematics_2WD_param_init(&kinematics_param);
    bus_serial_init(0, &huart5, NULL, 0);

    uint16_t val[2];
    if (!modbus_rtu_get_input_regs(0, 1, 5, 1, val)) {
        printf("Failed to get protocol version\n");
        return;
    }

    if (val[0] < 2) {
        printf("Protocol version too low\n");
        return;
    }

    if (!modbus_rtu_set_holding_reg(0, 1, 0x13, 50)) {
        printf("Failed to set PID interval\n");
        return;
    }

    while (true) {
        kinematics_2WD_inverse(&kinematics[0], &kinematics_param);
        val[0] = kinematics[0].speed[0];
        val[1] = kinematics[0].speed[1];
        modbus_rtu_set_holding_regs(0, 1, 0, val, 2);
        LOS_TaskDelay(10);
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

void start_tasks(void) {
    uint32_t communication_task_id;
    uint32_t imu_task_id;
    uint32_t bumper_task_id;
    uint32_t controller_task_id;
    uint32_t kinematics_task_id;

    // kernel init
    LOS_KernelInit();

    LOS_MemInit(mem_pool, MEMORY_POOL_SIZE);

    // lock the task scheduling
    LOS_TaskLock();

    // create tasks
    // create_task(&communication_task_id, (TSK_ENTRY_FUNC)communication_task,
    //             "communication_task", 5, 4096);
    // create_task(&imu_task_id, (TSK_ENTRY_FUNC)imu_task,
    //             "imu_task", 5, 4096);
    // create_task(&bumper_task_id, (TSK_ENTRY_FUNC)bumper_task,
    //             "bumper_task", 6, 1024);
    create_task(&controller_task_id, (TSK_ENTRY_FUNC)controller_task,
                "controller_task", 6, 0x1000);
    create_task(&kinematics_task_id, (TSK_ENTRY_FUNC)kinematics_task,
                "kinematics_task", 5, 0x1000);

    // unlock the task scheduling
    LOS_TaskUnlock();

    // start scheduling
    LOS_Start();
}
