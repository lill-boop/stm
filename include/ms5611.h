/**
 * @file ms5611.h
 * @brief MS5611 气压传感器驱动
 * @note  用于定高飞行模式 (Altitude Hold)
 */

#ifndef __MS5611_H
#define __MS5611_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 配置 ==================== */
#define MS5611_I2C_ADDR     0x77    // CSB=GND → 0x77, CSB=VCC → 0x76

/* 过采样率 (OSR) - 影响精度和转换时间 */
typedef enum {
    MS5611_OSR_256  = 0x00,  // 0.6 ms
    MS5611_OSR_512  = 0x02,  // 1.2 ms
    MS5611_OSR_1024 = 0x04,  // 2.3 ms
    MS5611_OSR_2048 = 0x06,  // 4.5 ms
    MS5611_OSR_4096 = 0x08   // 9.0 ms (最高精度)
} MS5611_OSR_t;

/* ==================== 数据结构 ==================== */
typedef struct {
    float pressure;      // 气压 (Pa)
    float temperature;   // 温度 (°C)
    float altitude;      // 相对高度 (m), 以初始化时为0
    float altitudeRaw;   // 绝对高度 (m)
    float verticalSpeed; // 垂直速度 (m/s), 正=上升
    bool valid;          // 数据是否有效
} MS5611_Data_t;

/* ==================== API ==================== */

/**
 * @brief 初始化 MS5611
 * @return true=成功, false=未检测到传感器
 */
bool MS5611_Init(void);

/**
 * @brief 触发一次测量 (非阻塞)
 * @note  需要等待转换完成后调用 MS5611_Read
 */
void MS5611_StartConversion(void);

/**
 * @brief 更新数据 (状态机模式, 每次调用推进一步)
 * @note  典型调用频率: 50-100Hz
 * @return true=本次调用产生了新数据
 */
bool MS5611_Update(void);

/**
 * @brief 获取最新数据
 */
MS5611_Data_t* MS5611_GetData(void);

/**
 * @brief 重置基准高度 (将当前高度设为0)
 */
void MS5611_ResetAltitude(void);

/**
 * @brief 获取基准气压
 */
float MS5611_GetGroundPressure(void);

#endif /* __MS5611_H */
