#include "ps2.h"
#include "stm32f4xx_hal.h"
#include "los_tick.h"

#define PS2_CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define PS2_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#if defined LOSCFG_PS2_USE_SOFT_SPI
#define PS2_CLK_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define PS2_CLK_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define PS2_DO_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define PS2_DO_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define PS2_READ_DI() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#elif defined LOSCFG_PS2_USE_HARD_SPI
static SPI_HandleTypeDef ps2_spi;

static void ps2_spi_init() {
    // msp init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // hal spi init
    ps2_spi.Instance = SPI1;
    ps2_spi.Init.Mode = SPI_MODE_MASTER;
    ps2_spi.Init.Direction = SPI_DIRECTION_2LINES;
    ps2_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    ps2_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    ps2_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    ps2_spi.Init.NSS = SPI_NSS_SOFT;
    ps2_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    ps2_spi.Init.FirstBit = SPI_FIRSTBIT_LSB;
    ps2_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    ps2_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    ps2_spi.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&ps2_spi);
}
#endif

static uint8_t ps2_data[9];
volatile PS2_State ps2_state;

static void ps2_decode() {
    uint8_t temp = ~ps2_data[3];
    ps2_state.Key_Select = (temp >> 0) & 0x01; //选择键
    ps2_state.Key_Start = (temp >> 3) & 0x01;  //开始键

    //左侧按键
    ps2_state.Key_L_Up = (temp >> 4) & 0x01;
    ps2_state.Key_L_Right = (temp >> 5) & 0x01;
    ps2_state.Key_L_Down = (temp >> 6) & 0x01;
    ps2_state.Key_L_Left = (temp >> 7) & 0x01;

    temp = ~ps2_data[4];
    //后侧按键
    ps2_state.Key_L2 = (temp >> 0) & 0x01;
    ps2_state.Key_R2 = (temp >> 1) & 0x01;
    ps2_state.Key_L1 = (temp >> 2) & 0x01;
    ps2_state.Key_R1 = (temp >> 3) & 0x01;

    //右侧按键
    ps2_state.Key_R_Up = (temp >> 4) & 0x01;
    ps2_state.Key_R_Right = (temp >> 5) & 0x01;
    ps2_state.Key_R_Down = (temp >> 6) & 0x01;
    ps2_state.Key_R_Left = (temp >> 7) & 0x01;

    if (ps2_data[1] == 0x41) {
        //无灯模式(摇杆值八向)
        ps2_state.Mode = PS2_DIGITAL;
        ps2_state.Rocker_LX = 127 * (ps2_state.Key_L_Right - ps2_state.Key_L_Left);
        ps2_state.Rocker_LY = 127 * (ps2_state.Key_L_Up - ps2_state.Key_L_Down);

        ps2_state.Rocker_RX = 127 * (ps2_state.Key_R_Right - ps2_state.Key_R_Left);
        ps2_state.Rocker_RY = 127 * (ps2_state.Key_R_Up - ps2_state.Key_R_Down);
    } else if (ps2_data[1] == 0x73) {
        //红灯模式(摇杆值模拟)
        ps2_state.Mode = PS2_ANALOG;
        //摇杆按键
        ps2_state.Key_Rocker_Left = (~ps2_data[3] >> 1) & 0x01;
        ps2_state.Key_Rocker_Right = (~ps2_data[3] >> 2) & 0x01;

        //摇杆值
        ps2_state.Rocker_LX = ps2_data[7] - 0x80;
        ps2_state.Rocker_LY = -1 - (ps2_data[8] - 0x80);
        ps2_state.Rocker_RX = ps2_data[5] - 0x80;
        ps2_state.Rocker_RY = -1 - (ps2_data[6] - 0x80);
    } else {
        ps2_state.Mode = PS2_INVALID;
    }
}

static void ps2_swap_byte(uint8_t tx_byte, uint8_t* rx_byte) {
#if defined LOSCFG_PS2_USE_HARD_SPI
    HAL_SPI_TransmitReceive(&ps2_spi, &tx_byte, rx_byte, 1, 0xFFFF);
#elif defined LOSCFG_PS2_USE_SOFT_SPI
    uint8_t rx = 0;
    for (int i = 0; i < 8; i++) {
        if (tx_byte & 0x01)
            PS2_DO_HIGH();
        else
            PS2_DO_LOW();

        tx_byte >>= 1;
        PS2_CLK_HIGH();
        LOS_UDelay(1);
        PS2_CLK_LOW();
        rx >>= 1;
        rx |= (PS2_READ_DI() << 7);
        LOS_UDelay(1);
        PS2_CLK_HIGH();
        LOS_UDelay(1);
        *rx_byte = rx;
    }
#endif
    LOS_UDelay(10);
}

void ps2_init() {
#if defined LOSCFG_PS2_USE_SOFT_SPI
#elif defined LOSCFG_PS2_USE_HARD_SPI
    ps2_spi_init();
#endif
}

void ps2_read_data() {
    PS2_CS_LOW();  //拉低，开始通讯

    ps2_swap_byte(0x01, &ps2_data[0]);
    ps2_swap_byte(0x42, &ps2_data[1]);

    for(int i = 2; i < 9; i++) {
        ps2_swap_byte(0x00, &ps2_data[i]);
    }

    PS2_CS_HIGH();  //拉高
    ps2_decode();
}
