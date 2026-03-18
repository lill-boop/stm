/**
 * @file drdy_imu.h
 * @brief DRDY 中断驱动 IMU 模块
 * @note  2kHz 中断驱动采样 + SPI DMA 读取
 * 
 * 架构:
 * ICM42688 DRDY (2kHz) → PB1 EXTI → SPI DMA Burst → 回调处理
 */

#ifndef __DRDY_IMU_H
#define __DRDY_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 配置 ==================== */

/* DRDY 引脚 */
#define DRDY_GPIO_PORT      GPIOB
#define DRDY_GPIO_PIN       GPIO_PIN_1
#define DRDY_EXTI_IRQn      EXTI1_IRQn

/* 控制频率 */
#define IMU_SAMPLE_RATE_HZ  2000
#define PWM_OUTPUT_RATE_HZ  400
#define PWM_DIVIDER         (IMU_SAMPLE_RATE_HZ / PWM_OUTPUT_RATE_HZ)  // 5

/* ==================== 数据结构 ==================== */

typedef struct {
    float gyro[3];      // 滤波后陀螺仪 (deg/s)
    float accel[3];     // 加速度 (g)
    float gyro_raw[3];  // 原始陀螺仪
    float temperature;   // 温度
    uint32_t timestamp;  // 时间戳 (us)
    float dt;           // 采样间隔 (s)
} DRDY_IMU_Data_t;

/* ==================== 统计信息 ==================== */

typedef struct {
    uint32_t sampleCount;       // 采样计数
    uint32_t maxLoopTime_us;    // 最大循环时间 (dt)
    uint32_t minLoopTime_us;    // 最小循环时间
    uint32_t avgLoopTime_us;    // 平均循环时间
    uint32_t maxExecTime_us;    // 最大执行时间 (ISR耗时)
    uint32_t avgExecTime_us;    // 平均执行时间
    uint32_t p99ExecTime_us;    // P99 执行时间
    uint32_t isrOverbudget;     // ISR 超预算计数 (>100us)
    uint32_t bad_dt_count;      // 异常 dt 计数
    uint32_t missed_samples;    // 丢帧/过载计数
    uint32_t spiErrors;         // SPI 错误计数
} DRDY_Stats_t;

/* ==================== API ==================== */

/**
 * @brief 初始化 DRDY 中断驱动
 * @note  配置 EXTI, SPI DMA, ICM42688 DRDY 输出
 */
void DRDY_IMU_Init(void);

/**
 * @brief 启动 DRDY 中断
 */
void DRDY_IMU_Start(void);

/**
 * @brief 停止 DRDY 中断
 */
void DRDY_IMU_Stop(void);

/**
 * @brief 获取最新 IMU 数据
 */
DRDY_IMU_Data_t* DRDY_IMU_GetData(void);

/**
 * @brief 获取统计信息
 */
DRDY_Stats_t* DRDY_IMU_GetStats(void);

/**
 * @brief [Optimal-G] 获取 Gyro 环形缓冲区 (用于频谱分析)
 * @param count 输出拷贝的样本数量 (最大 256)
 * @param buffer 输出缓冲区指针
 */
void DRDY_IMU_GetGyroBuffer(uint16_t count, float *buffer);

/**
 * @brief [Optimal-A] 计算 p99 执行时间 (基于环形缓冲)
 * @note 在主循环 5Hz 调用，更新 stats.p99ExecTime_us
 */
void DRDY_IMU_CalcP99(void);

/**
 * @brief 检查是否应该更新 PWM (400Hz)
 */
bool DRDY_IMU_ShouldUpdatePWM(void);

/**
 * @brief 用户回调: 2kHz 控制循环
 * @note  需要用户实现, 在 DMA 完成后调用
 */
extern void DRDY_FlightControl_Callback(DRDY_IMU_Data_t *data, float dt);

/* ==================== 调试与配置API ==================== */

typedef struct {
    float dt_s;
    float gyro_dps[3];
    float acc_g[3];
    float acc_norm;
    float acc_weight;
    float beta;
    float quat[4];
} IMU_Debug_Data_t;

extern volatile IMU_Debug_Data_t imuDebugData;  // ISR更新
extern bool imuDebugActive;            // 控制是否收集

void DRDY_IMU_SetAxisMapping(uint8_t map[3], int8_t sign[3]);
void DRDY_IMU_GetAxisMapping(uint8_t map[3], int8_t sign[3]);

void DRDY_IMU_SetGyroBias(float bias[3]);
void DRDY_IMU_GetGyroBias(float bias[3]);

/**
 * @brief 阻塞式陀螺仪校准 (2秒)
 */
void DRDY_IMU_CalibrateGyro(void);

/**
 * @brief 设置 Notch1 滤波器参数
 */
void DRDY_IMU_SetNotchFreq(float centerFreq, float Q);

/**
 * @brief [FFT] 设置 Notch2 滤波器参数
 */
void DRDY_IMU_SetNotchFreq2(float centerFreq, float Q);

/**
 * @brief [Phase-2.2] 设置动态陀螺仪LPF截止频率 (基于油门)
 * @param throttlePercent 油门百分比 0-100
 */
void DRDY_IMU_SetDynamicGyroLPF(float throttlePercent);

#endif /* __DRDY_IMU_H */
