/**
 * @file system_config.h
 * @brief 系统配置头文件 - STM32G431飞控项目
 */

#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "stm32g4xx_hal.h"

/* ==================== 系统配置 ==================== */
#define SYSTEM_CLOCK_FREQ   170000000   // 170MHz (STM32G431最高频率)

/* ==================== 调试串口配置 ==================== */
#define DEBUG_BAUDRATE      115200      // 修改为115200以适配BLE蓝牙模块

/* ==================== IMU配置 ==================== */
#define IMU_ODR             ODR_2KHZ    // 2000Hz采样率 (最优延迟配置)
#define IMU_GYRO_FS         GYRO_FS_2000DPS   // ±2000°/s (飞控常用)
#define IMU_ACCEL_FS        ACCEL_FS_16G      // ±16g

/* ==================== 主循环频率 ==================== */
#define MAIN_LOOP_FREQ_HZ   1000        // 目标1000Hz主循环

/* ==================== 函数声明 ==================== */
void SystemClock_Config(void);
void Error_Handler(void);

#endif /* __SYSTEM_CONFIG_H */
