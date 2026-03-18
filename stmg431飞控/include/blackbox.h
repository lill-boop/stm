/**
 * @file blackbox.h
 * @brief 最小 Blackbox 日志系统
 * @note  100Hz UART 二进制输出，用于频谱分析和调参
 */

#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 配置 ==================== */

#define BLACKBOX_RATE_HZ    100     // 日志频率
#define BLACKBOX_FRAME_SIZE 32      // 帧大小 (bytes)

/* 帧头 */
#define BLACKBOX_HEADER_1   0xAA
#define BLACKBOX_HEADER_2   0x55

/* ==================== 日志数据结构 ==================== */

#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];         // 0xAA 0x55
    uint16_t timestamp;         // ms (低16位)
    int16_t  gyro_raw[3];       // 原始陀螺仪 (0.1 deg/s)
    int16_t  gyro_filt[3];      // 滤波后陀螺仪
    int16_t  dterm[3];          // D 项输出
    uint16_t motor[4];          // 电机 PWM
    uint16_t throttle;          // 油门
    uint8_t  loop_dt;           // 循环时间 (10us 单位)
    uint8_t  checksum;          // XOR 校验
} BlackboxFrame_t;
#pragma pack(pop)

/* ==================== API ==================== */

/**
 * @brief 初始化 Blackbox
 */
void Blackbox_Init(void);

/**
 * @brief 记录一帧数据 (在 2kHz 循环中调用)
 * @note  内部自动分频到 100Hz
 */
void Blackbox_Log(float *gyro_raw, float *gyro_filt, float *dterm, 
                  uint16_t *motor, uint16_t throttle, uint32_t dt_us);

/**
 * @brief 启用/禁用 Blackbox
 */
void Blackbox_Enable(bool enable);

/**
 * @brief 检查是否启用
 */
bool Blackbox_IsEnabled(void);

#endif /* __BLACKBOX_H */
