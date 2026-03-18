/**
 * @file soft_spi_test.h
 * @brief 软件SPI测试头文件
 */

#ifndef __SOFT_SPI_TEST_H
#define __SOFT_SPI_TEST_H

#include "stm32g4xx_hal.h"

void SoftSPI_Init(void);
uint8_t SoftSPI_TransferByte(uint8_t txData);
uint8_t SoftSPI_ReadReg(uint8_t reg);
void SoftSPI_SetMode(uint8_t mode);
void SoftSPI_GPIOTest(void);

#endif
