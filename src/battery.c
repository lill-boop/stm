/**
 * @file battery.c
 * @brief 电池电压检测实现
 */

#include "battery.h"
#include "stm32g4xx_hal.h"

/* ADC句柄 */
static ADC_HandleTypeDef hadc1;

/* ==================== API实现 ==================== */

/**
 * @brief 初始化ADC1检测电池
 * @note  PA1 -> ADC1_IN2
 */
void Battery_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 时钟使能 */
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置GPIO (PA1) 为模拟输入 */
    GPIO_InitStruct.Pin = BATTERY_ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATTERY_ADC_PORT, &GPIO_InitStruct);
    
    /* 配置ADC1 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        while(1);
    }
    
    /* 配置通道2 (PA1) */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // 长采样时间以稳定读数
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        while(1);
    }
    
    /* 校准ADC */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

/**
 * @brief 读取电池电压
 */
float Battery_GetVoltage(void)
{
    /* 启动转换 */
    HAL_ADC_Start(&hadc1);
    
    /* 等待转换完成 */
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        
        /* 计算电压: ADC值 / 4095 * 3.3V * 分压比 */
        float v_pin = (float)raw / 4095.0f * ADC_REF_VOLTAGE;
        float v_bat = v_pin * ((BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2);
        
        return v_bat;
    }
    
    return 0.0f;
}

/**
 * @brief 检查低压
 */
uint8_t Battery_IsLowVoltage(void)
{
    float v = Battery_GetVoltage();
    return (v > 0.5f && v < BATTERY_WARN_VOLT); // >0.5排除未接电池情况
}
