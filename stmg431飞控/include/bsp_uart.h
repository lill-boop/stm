/**
 * @file bsp_uart.h
 * @brief 串口驱动头文件 - STM32G431飞控项目
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32g4xx_hal.h"
#include <stdio.h>

/* ==================== UART2 引脚定义 ==================== */
// 使用UART2作为调试串口 (通过USB-TTL连接电脑)
#define DEBUG_UART_TX_PIN   GPIO_PIN_2
#define DEBUG_UART_TX_PORT  GPIOA
#define DEBUG_UART_RX_PIN   GPIO_PIN_3
#define DEBUG_UART_RX_PORT  GPIOA

/* ==================== 函数声明 ==================== */
void BSP_UART2_Init(uint32_t baudrate);
void BSP_UART_SendByte(uint8_t byte);
void BSP_UART_SendString(const char *str);
void BSP_UART_Printf(const char *fmt, ...);

#endif /* __BSP_UART_H */
