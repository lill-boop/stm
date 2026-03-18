/**
 * @file bsp_spi.c
 * @brief SPI驱动实现 - STM32G431飞控项目
 */

#include "bsp_spi.h"

/* SPI句柄 */
SPI_HandleTypeDef hspi1;

/**
 * @brief 初始化SPI1
 * @note  时钟频率约10MHz，适合ICM42688高速通信
 */
void BSP_SPI1_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置SPI引脚 PA5=SCK, PA6=MISO, PA7=MOSI */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);
    
    /* 配置CS引脚为普通GPIO输出 */
    gpio.Pin = ICM42688_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = 0;
    HAL_GPIO_Init(ICM42688_CS_PORT, &gpio);
    
    /* CS默认拉高（不选中） */
    ICM42688_CS_HIGH();
    
    /* 配置SPI1参数 - Mode 3 (CPOL=1, CPHA=1) */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;       // CPOL = 0
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;           // CPHA = 0 (SPI Mode 0)
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  // ~2.6MHz (Production Mode)
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        /* 初始化失败处理 */
        while(1);
    }
    
    /* STM32G4的SPI有FIFO，8位数据需要设置FIFO阈值为1/4 */
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_FRXTH);
}

/**
 * @brief SPI单字节收发
 * @param data 发送的字节
 * @return 接收到的字节
 */
uint8_t BSP_SPI1_TransmitReceive(uint8_t data)
{
    uint8_t rxData = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, 100);
    return rxData;
}

/**
 * @brief SPI发送多字节
 */
void BSP_SPI1_Transmit(uint8_t *pData, uint16_t size)
{
    HAL_SPI_Transmit(&hspi1, pData, size, 100);
}

/**
 * @brief SPI接收多字节
 */
void BSP_SPI1_Receive(uint8_t *pData, uint16_t size)
{
    HAL_SPI_Receive(&hspi1, pData, size, 100);
}
