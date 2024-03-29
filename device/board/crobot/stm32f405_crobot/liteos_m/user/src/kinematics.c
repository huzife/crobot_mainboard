#include "kinematics.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>

// static TIM_HandleTypeDef kinematics_tim;
TIM_HandleTypeDef kinematics_tim;

Kinematics k_inverse = {0};
Kinematics k_forward = {0};
Odometry odometry = {0};

static void kinematics_odom_timer_setup() {
    __HAL_RCC_TIM7_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    kinematics_tim.Instance = TIM7;
    kinematics_tim.Init.Prescaler = 8400-1;
    kinematics_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    kinematics_tim.Init.Period = 65535;
    kinematics_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&kinematics_tim) != HAL_OK)
        Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&kinematics_tim, &sMasterConfig) != HAL_OK)
        Error_Handler();
    __HAL_TIM_CLEAR_FLAG(&kinematics_tim, TIM_FLAG_UPDATE);
}

void kinematics_init() {
    kinematics_odom_timer_setup();
    HAL_TIM_Base_Start_IT(&kinematics_tim);
}

void kinematics_update_odom() {
    // get interval, reset counter
    float interval = __HAL_TIM_GET_COUNTER(&kinematics_tim) / 10000.0;
    __HAL_TIM_SET_COUNTER(&kinematics_tim, 0);

    // update odom
    float vx = k_forward.velocity.linear_x;
    float vy = k_forward.velocity.linear_y;
    float angular = k_forward.velocity.angular_z;

    float dir = odometry.direction;
    float delta_x = (vx * cosf(dir) - vy * sinf(dir)) * interval;
    float delta_y = (vx * sinf(dir) + vy * cosf(dir)) * interval;
    float delta_yaw = angular * interval;

    odometry.position_x += delta_x;
    odometry.position_y += delta_y;
    odometry.direction += delta_yaw;
}

Odometry kinematics_get_odom() {
    return odometry;
}

void kinematics_set_target_velocity(Velocity velocity) {
    k_inverse.velocity = velocity;
}

Velocity kinematics_get_current_velocity() {
    return k_forward.velocity;
}

void kinematics_set_current_motor_speed(uint16_t* speeds) {
    for (int i = 0; i < WHEEL_NUM; i++) {
        k_forward.speed[i] = speeds[i];
    }
}

uint16_t* kinematics_get_target_motor_speed() {
    return k_inverse.speed;
}
