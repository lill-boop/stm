/**
 * @file system_config.c
 * @brief 系统时钟和初始化配置
 */

#include "system_config.h"

/**
 * @brief 配置系统时钟到170MHz
 * @note  使用HSI(16MHz) -> PLL -> 170MHz
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 配置电压范围1以支持170MHz */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* 配置HSI和PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;    // 16MHz / 4 = 4MHz
    RCC_OscInitStruct.PLL.PLLN = 85;                // 4MHz * 85 = 340MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;    // 340MHz / 2 = 170MHz
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;    // 340MHz / 2 = 170MHz (SYSCLK)

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置系统时钟源和分频 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;    // HCLK = 170MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     // APB1 = 170MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     // APB2 = 170MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 错误处理函数
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // 可以在这里添加LED闪烁指示错误
    }
}

/**
 * @brief SysTick中断处理
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief HardFault处理
 */
void HardFault_Handler(void)
{
    while(1);
}

void MemManage_Handler(void)
{
    while(1);
}

void BusFault_Handler(void)
{
    while(1);
}

void UsageFault_Handler(void)
{
    while(1);
}
