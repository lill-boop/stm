/**
 * @file bsp_spi.h
 * @brief SPI驱动头文件 - STM32G431飞控项目
 */

#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "stm32g4xx_hal.h"

/* ==================== SPI1 引脚定义 ==================== */
// SPI1 用于 ICM42688
#define SPI1_SCK_PIN        GPIO_PIN_5
#define SPI1_SCK_PORT       GPIOA
#define SPI1_MISO_PIN       GPIO_PIN_6
#define SPI1_MISO_PORT      GPIOA
#define SPI1_MOSI_PIN       GPIO_PIN_7
#define SPI1_MOSI_PORT      GPIOA

// ICM42688 片选引脚
#define ICM42688_CS_PIN     GPIO_PIN_4
#define ICM42688_CS_PORT    GPIOA

// ICM42688 中断引脚 (可选)
#define ICM42688_INT_PIN    GPIO_PIN_1
#define ICM42688_INT_PORT   GPIOB

/* ==================== 片选控制宏 ==================== */
#define ICM42688_CS_LOW()   HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET)
#define ICM42688_CS_HIGH()  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET)

/* ==================== 函数声明 ==================== */
void BSP_SPI1_Init(void);
uint8_t BSP_SPI1_TransmitReceive(uint8_t data);
void BSP_SPI1_Transmit(uint8_t *pData, uint16_t size);
void BSP_SPI1_Receive(uint8_t *pData, uint16_t size);

#endif /* __BSP_SPI_H */
