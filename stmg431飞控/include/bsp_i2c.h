/**
 * @file bsp_i2c.h
 * @brief I2C 驱动封装 - STM32G431飞控项目
 * @note  使用 I2C1 (PB8=SCL, PB9=SDA)
 */

#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include <stdint.h>
#include <stdbool.h>

/* [Phase-2] 降低 I2C 超时，防止主循环阻塞 */
#define I2C_TIMEOUT_MS  5

/* ==================== API ==================== */

/**
 * @brief 初始化 I2C1 (PB8=SCL, PB9=SDA)
 */
void BSP_I2C1_Init(void);

/**
 * @brief 写入数据到 I2C 设备
 * @param devAddr 7位设备地址 (不左移)
 * @param regAddr 寄存器地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return true=成功
 */
bool BSP_I2C1_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len);

/**
 * @brief 从 I2C 设备读取
 * @param devAddr 7位设备地址
 * @param regAddr 寄存器地址
 * @param data 接收缓冲区
 * @param len 读取长度
 * @return true=成功
 */
bool BSP_I2C1_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len);

/**
 * @brief 发送命令 (无寄存器地址)
 */
bool BSP_I2C1_WriteCmd(uint8_t devAddr, uint8_t cmd);

/**
 * @brief 读取数据 (无寄存器地址)
 */
bool BSP_I2C1_ReadData(uint8_t devAddr, uint8_t *data, uint16_t len);

/* ==================== 诊断功能 ==================== */

/**
 * @brief 检查 I2C 总线状态
 * @return 0=正常, 1=SCL低, 2=SDA低, 3=都低
 */
uint8_t BSP_I2C1_CheckBusState(void);

/**
 * @brief 总线恢复 (发送9个时钟脉冲)
 * @return true=恢复成功
 */
bool BSP_I2C1_BusRecovery(void);

/**
 * @brief 扫描 I2C 总线
 * @param foundAddrs 返回找到的地址 (最多16个)
 * @return 设备数量
 */
uint8_t BSP_I2C1_Scan(uint8_t *foundAddrs);

/**
 * @brief 完整诊断 (输出到串口)
 */
void BSP_I2C1_Diagnose(void);

#endif /* __BSP_I2C_H */
