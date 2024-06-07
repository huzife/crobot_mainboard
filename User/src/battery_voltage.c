#include "battery_voltage.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BATTERY_VOLTAGE_WINDOW_SIZE 8
static uint32_t adc_value_accu[BATTERY_VOLTAGE_WINDOW_SIZE];
static uint32_t adc_value_sum;
static uint32_t adc_value_index;
static bool is_window_full;

static ADC_HandleTypeDef battery_voltage_adc;
static float battery_voltage;

static void battery_voltage_adc_init() {
    // msp init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // hal adc init
    ADC_ChannelConfTypeDef sConfig = {0};
    battery_voltage_adc.Instance = ADC1;
    battery_voltage_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    battery_voltage_adc.Init.Resolution = ADC_RESOLUTION_12B;
    battery_voltage_adc.Init.ScanConvMode = DISABLE;
    battery_voltage_adc.Init.ContinuousConvMode = DISABLE;
    battery_voltage_adc.Init.DiscontinuousConvMode = DISABLE;
    battery_voltage_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    battery_voltage_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    battery_voltage_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    battery_voltage_adc.Init.NbrOfConversion = 1;
    battery_voltage_adc.Init.DMAContinuousRequests = DISABLE;
    battery_voltage_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&battery_voltage_adc);

    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&battery_voltage_adc, &sConfig);
}

void battery_voltage_init() {
    battery_voltage_adc_init();
    adc_value_index = 0;
    adc_value_sum = 0;
    is_window_full = false;
    battery_voltage = 0.0f;
}

float battery_get_voltage() {
    return battery_voltage;
}

static void battery_voltage_rolling_mean(uint32_t value) {
    if (is_window_full)
        adc_value_sum -= adc_value_accu[adc_value_index];

    adc_value_sum += value;
    adc_value_accu[adc_value_index++] = value;

    if (adc_value_index == BATTERY_VOLTAGE_WINDOW_SIZE) {
        adc_value_index = 0;
        if (!is_window_full)
            is_window_full = true;
    }

    // calculate and set battery_voltage
    uint32_t size = is_window_full ? BATTERY_VOLTAGE_WINDOW_SIZE
                                   : adc_value_index;
    float adc_value = (float)adc_value_sum / size;
    battery_voltage = adc_value * 3.3f * (133.0f / 33.0f) / ((1 << 12) - 1);
}

void battery_voltage_update() {
    HAL_ADC_Start(&battery_voltage_adc);
    HAL_ADC_PollForConversion(&battery_voltage_adc, 1000);
    battery_voltage_rolling_mean(HAL_ADC_GetValue(&battery_voltage_adc));
}
