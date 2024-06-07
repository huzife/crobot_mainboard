#include "battery_voltage.h"
#include "bumper.h"
#include "crobot_debug.h"
#include "host_com.h"
#include "icm42605.h"
#include "kinematics.h"
#include "modbus_rtu.h"
#include "ps2.h"
#include "ultrasonic.h"
#include "vel_mux.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t host_com_task_handler;
TaskHandle_t bumper_task_handler;
TaskHandle_t controller_task_handler;
TaskHandle_t kinematics_task_handler;
TaskHandle_t ultrasonic_task_handler;
TaskHandle_t imu_task_handler;
TaskHandle_t battery_voltage_task_handler;

static void host_com_task(void* param) {
    if (!host_com_init(128)) {
        PRINT_ERR("Host com init failed\n");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        if (!host_com_parse())
            vTaskDelay(1);
    }
}

static void bumper_task(void* param) {
    int vel_id = vel_mux_register(0, 200);
    if (vel_id < 0) {
        PRINT_ERR("Register bumper velocity failed\n");
        vTaskDelete(NULL);
        return;
    }
    Velocity_Message velocity = {vel_id, {0.0f, 0.0f, 0.0f}};
    bumper_init();

    while (true) {
        Bumper_State state = bumper_check();
        if (state.left || state.front || state.right) {
            if (state.left) {
                velocity.velocity.linear_x -= 0.1f;
                velocity.velocity.angular_z -= 0.4f;
            }
            if (state.front) {
                velocity.velocity.linear_x -= 0.1f;
            }
            if (state.right) {
                velocity.velocity.linear_x -= 0.1f;
                velocity.velocity.angular_z += 0.4f;
            }

            // set velocity
            for (int i = 0; i < 20; i++) {
                vel_mux_set_velocity(velocity);
                vTaskDelay(100);
            }

            // stop
            velocity.velocity.linear_x = 0.0f;
            velocity.velocity.angular_z = 0.0f;
            vel_mux_set_velocity(velocity);
        }

        vTaskDelay(10);
    }
}

static void controller_task(void* param) {
    ps2_init();
    int vel_id = vel_mux_register(1, 100);
    if (vel_id < 0) {
        PRINT_ERR("Register controller failed\n");
        vTaskDelete(NULL);
        return;
    }

    Velocity_Message velocity = {vel_id, {0.0f, 0.0f, 0.0f}};

    while (true) {
        ps2_read_data();
        if (ps2_state.Mode) {
            velocity.velocity.linear_x = 0.3f * ps2_state.Rocker_LY / 128;
            velocity.velocity.angular_z = -1.0f * ps2_state.Rocker_RX / 128;
            vel_mux_set_velocity(velocity);
            vTaskDelay(50);
        } else {
            vTaskDelay(1000);
        }
    }
}

static void kinematics_task(void* param) {
    uint16_t proto_rev;
    if (!modbus_get_input_regs(0, 1, 0xFF, 1, &proto_rev)) {
        PRINT_ERR("Failed to read protocol revision\n");
        vTaskDelete(NULL);
        return;
    } else if (proto_rev < 3) {
        PRINT_ERR("Protocol revision too low\n");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        if (kinematics_start()) {
            kinematics_update_info();
            kinematics_handle_velocity();
            kinematics_end();
        }

        vTaskDelay(50);
    }
}

static void ultrasonic_task(void* param) {
    if (!ultrasonic_init()) {
        PRINT_ERR("Failed to init ultrasonic\n");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        ultrasonic_update_range();
        vTaskDelay(50);
    }
}

static void imu_task(void* param) {
    icm42605_init();
    int cnt = 0;
    while (true) {
        // imu data update rate: 100Hz
        icm42605_update_data();

        // imu temperature update rate: 1Hz
        if (++cnt == 50) {
            icm42605_update_temperature();
            cnt = 0;
        }

        vTaskDelay(10);
    }
}

static void battery_voltage_task(void* param) {
    battery_voltage_init();
    while (true) {
        battery_voltage_update();
        vTaskDelay(1000);
    }
}

static void crobot_init() {
    modbus_init();
    kinematics_init();
    vel_mux_init(8);
}

void crobot_start() {
    crobot_init();

    xTaskCreate(host_com_task, "host_com_task", 0x100, NULL, 6, &host_com_task_handler);
    xTaskCreate(bumper_task, "bumper_task", 0x100, NULL, 6, &bumper_task_handler);
    xTaskCreate(controller_task, "controller_task", 0x100, NULL, 6, &controller_task_handler);
    xTaskCreate(kinematics_task, "kinematics_task", 0x100, NULL, 6, &kinematics_task_handler);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 0x100, NULL, 6, &ultrasonic_task_handler);
    xTaskCreate(imu_task, "imu_task", 0x100, NULL, 6, &imu_task_handler);
    xTaskCreate(battery_voltage_task, "battery_voltage_task", 0x100, NULL, 6, &battery_voltage_task_handler);

    vTaskStartScheduler();
}
