#include "battery_voltage.h"
#include "los_atomic.h"

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
    battery_voltage = 0.0;
}

float battery_get_voltage() {
    int voltage = LOS_AtomicRead((const Atomic*)&battery_voltage);

    return *(float*)&voltage;
}

void battery_voltage_update() {
    HAL_ADC_Start(&battery_voltage_adc);
    HAL_ADC_PollForConversion(&battery_voltage_adc, 1000);
    uint32_t adc_value = HAL_ADC_GetValue(&battery_voltage_adc);
    float voltage = adc_value * 3.3 * (133.0 / 33.0) / ((1 << 12) - 1);
    LOS_AtomicSet((Atomic*)&battery_voltage, *(int*)&voltage);
}
