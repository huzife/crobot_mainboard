#include "bumper.h"
#include "main.h"

#define BUMPER_LEFT_MASK (1 << 0)
#define BUMPER_FRONT_MASK (1 << 1)
#define BUMPER_RIGHT_MASK (1 << 2)

#define BUMPER_HIT_LEFT() !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define BUMPER_HIT_FRONT() !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define BUMPER_HIT_RIGHT() !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)

static uint8_t state = 0;

void bumper_init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

Bumper_State bumper_check() {
    Bumper_State bumper_state;

    // left
    if (BUMPER_HIT_LEFT()) {
        bumper_state.left = state & BUMPER_LEFT_MASK;
        state |= BUMPER_LEFT_MASK;
    } else {
        state &= ~BUMPER_LEFT_MASK;
    }

    // front
    if (BUMPER_HIT_FRONT()) {
        bumper_state.front = state & BUMPER_FRONT_MASK;
        state |= BUMPER_FRONT_MASK;
    } else {
        state &= ~BUMPER_FRONT_MASK;
    }

    // right
    if (BUMPER_HIT_RIGHT()) {
        bumper_state.right = state & BUMPER_RIGHT_MASK;
        state |= BUMPER_RIGHT_MASK;
    } else {
        state &= ~BUMPER_RIGHT_MASK;
    }

    return bumper_state;
}
