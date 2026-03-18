/**
 * @file bsp_uart.c
 * @brief 串口驱动实现 - STM32G431飞控项目
 */

#include "bsp_uart.h"
#include "cli.h"
#include <stdarg.h>
#include <string.h>

/* UART句柄 */
UART_HandleTypeDef huart2;

/* 接收缓冲 */
static uint8_t rxByte;

/**
 * @brief 初始化UART2用于调试输出
 * @param baudrate 波特率 (推荐921600或2000000)
 */
void BSP_UART2_Init(uint32_t baudrate)
{
    /* 使能时钟 */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置UART引脚 PA2=TX, PA3=RX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = DEBUG_UART_TX_PIN | DEBUG_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    
    /* 配置UART参数 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = baudrate;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while(1);
    }
    
    /* 使能接收中断 (优先级降低，防止打断控制回路) */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
}

/**
 * @brief 发送单字节
 */
void BSP_UART_SendByte(uint8_t byte)
{
    HAL_UART_Transmit(&huart2, &byte, 1, 10);
}

/**
 * @brief 发送字符串
 */
void BSP_UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

/**
 * @brief 格式化打印（类似printf）
 */
void BSP_UART_Printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    BSP_UART_SendString(buffer);
}

/**
 * @brief USART2中断处理
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart2.Instance->RDR & 0xFF);
        CLI_ProcessChar((char)ch);
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
    HAL_UART_IRQHandler(&huart2);
}

/* 重定向printf到串口 */
#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
    return len;
}
#endif

