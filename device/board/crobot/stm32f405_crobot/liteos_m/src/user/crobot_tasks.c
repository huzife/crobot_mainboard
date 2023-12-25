#include "share_ware.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "los_memory.h"
#include "los_queue.h"
#include "los_task.h"
#include "los_interrupt.h"
#include <string.h>

void task_delay(uint32_t ms) {
    LOS_TaskDelay(LOS_MS2Tick(ms));
}

void communication_task(void) {
    printf("communication_task\n");
    squeue_init(&data_queue);
    data_parser_init(&parser);
    HAL_UART_Receive_IT(&huart1, &com_rx_data, 1);

    for (;;) {
        task_delay(1);

        // 接收并解析数据帧
        while (!parser.flag) {
            uint8_t data;
            if (s_pop(&data_queue, &data))
                parse(&parser, data);
            else
                task_delay(1);
        }

        // 处理数据
        while (huart1.gState != HAL_UART_STATE_READY);

        parser.flag = 0;
        process_data((uint8_t*)parser.buf);
    }
}

void imu_task(void) {
    printf("imu_task\n");
    icm_init();
    for (;;) {
        icm_get_raw_data(&icm_raw_data);
        task_delay(10);
    }
}

void gpio_test_task(void) {
    printf("gpio_test_task\n");
    for (;;) {
        // uint8_t cnt = 0;
        // if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
        //     ++cnt;
        // if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
        //     ++cnt;
        // if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
        //     ++cnt;

        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, !(cnt % 2));
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, cnt < 2);
        // task_delay(10);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
        task_delay(200);
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
    uint32_t gpio_test_task_id;

    LOS_KernelInit();

    LOS_TaskLock();
    create_task(&communication_task_id, communication_task, "communication_task", 5, 4096);
    create_task(&imu_task_id, imu_task, "imu_task", 5, 4096);
    create_task(&gpio_test_task_id, gpio_test_task, "gpio_test_task", 6, 1024);
    LOS_TaskUnlock();

    LOS_Start();
}
