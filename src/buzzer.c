/**
 * @file buzzer.c
 * @brief 蜂鸣器实现
 */

#include "buzzer.h"
#include "stm32g4xx_hal.h"

void Buzzer_Init(void)
{
    /* PC14时钟使能 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    
    /* 默认关闭 (假设高电平响，取决于电路，通常三极管驱动是高电平导通) */
    Buzzer_Off();
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Buzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Beep(uint8_t count, uint32_t period)
{
    for (int i = 0; i < count; i++) {
        Buzzer_On();
        HAL_Delay(period);
        Buzzer_Off();
        HAL_Delay(period);
    }
}
