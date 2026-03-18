/**
 * @file config.h
 * @brief 参数存储模块
 * @note  使用STM32内部Flash模拟EEPROM，保存PID参数
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

/* 配置结构体 (需要保存的数据) */
typedef struct {
    uint32_t version;       // 版本号 (用于校验)
    FlightPID_t pid;        // PID参数
    float madgwickBeta;     // 姿态解算参数
    
    /* IMU 配置 */
    uint8_t axisMap[3];     // 轴映射: 0=X, 1=Y, 2=Z
    int8_t axisSign[3];     // 轴符号: 1 or -1
    float gyroBias[3];      // 陀螺仪零偏 (dps)
    
    uint32_t checksum;      // 校验和
} Config_t;

/* ==================== API函数 ==================== */

/**
 * @brief 从Flash加载配置
 * @return true=加载成功, false=加载失败(使用默认值)
 */
bool Config_Load(FlightPID_t *pid);

/**
 * @brief 保存配置到Flash
 */
void Config_Save(FlightPID_t *pid);

/**
 * @brief 恢复默认参数
 */
void Config_ResetDefaults(FlightPID_t *pid);

#endif /* __CONFIG_H */
