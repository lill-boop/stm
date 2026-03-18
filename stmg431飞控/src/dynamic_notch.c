/**
 * @file dynamic_notch.c
 * @brief 动态 Notch 滤波器自适应调整 (Goertzel 峰值检测)
 * @note  基于 STMG431 FPU 硬件加速，运行于 50Hz 任务
 */

#include "drdy_imu.h"
#include <math.h>

/* Goertzel 算法参数 */
#define GYRO_BUFFER_SIZE 256
#define SAMPLE_RATE      2000.0f
#define SCAN_START_HZ    60.0f
#define SCAN_END_HZ      350.0f
#define SCAN_STEP_HZ     10.0f

#define MIN_PEAK_ENERGY  1000.0f  // 只有超过此能量才更新 filter
#define SMOOTH_FACTOR    0.1f     // 频率平滑因子

/* 调试数据导出 */
float debug_peak_freq = 0.0f;
float debug_peak_energy = 0.0f;
uint32_t notch_update_count = 0;

/* 前一次使用的中心频率 (用于平滑) */
static float current_notch_hz = 150.0f;

/* Goertzel 单频点能量计算 */
static float Goertzel_MagnitudeSquared(float target_freq, float *data, int num_samples)
{
    float omega = (2.0f * 3.14159265f * target_freq) / SAMPLE_RATE;
    float sine = sinf(omega);
    float cosine = cosf(omega);
    float coeff = 2.0f * cosine;
    
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    
    for (int i = 0; i < num_samples; i++) {
        q0 = coeff * q1 - q2 + data[i];
        q2 = q1;
        q1 = q0;
    }
    
    /* 能量 = |y[N]|^2 */
    // 标准Goertzel输出复数功率，这里简化求幅度平方
    // power = q1^2 + q2^2 - q1*q2*coeff
    return (q1 * q1) + (q2 * q2) - (q1 * q2 * coeff);
}

/**
 * @brief 执行频谱扫描并更新 Notch
 * @note  建议在 50Hz 任务中调用 (每 3 次调用执行 1 次更新)
 */
void DynamicNotch_Update(void)
{
    static int divider = 0;
    divider++;
    if (divider < 3) return; // 50Hz / 3 = 16.6Hz 更新率
    divider = 0;
    
    /* 获取 Gyro 数据 */
    static float buffer[GYRO_BUFFER_SIZE];
    DRDY_IMU_GetGyroBuffer(GYRO_BUFFER_SIZE, buffer);
    
    /* 去直流 (简单的平均值扣除，减少低频泄漏) */
    float sum = 0.0f;
    for(int i=0; i<GYRO_BUFFER_SIZE; i++) sum += buffer[i];
    float mean = sum / GYRO_BUFFER_SIZE;
    for(int i=0; i<GYRO_BUFFER_SIZE; i++) buffer[i] -= mean;
    
    /* 频谱扫描找峰值 */
    float max_energy = 0.0f;
    float best_freq = current_notch_hz; // 默认保持
    
    for (float f = SCAN_START_HZ; f <= SCAN_END_HZ; f += SCAN_STEP_HZ) {
        float energy = Goertzel_MagnitudeSquared(f, buffer, GYRO_BUFFER_SIZE);
        
        if (energy > max_energy) {
            max_energy = energy;
            best_freq = f;
        }
    }
    
    /* 调试数据 */
    debug_peak_freq = best_freq;
    debug_peak_energy = max_energy;
    
    /* 只有当能量超过门限时才更新 */
    if (max_energy > MIN_PEAK_ENERGY) {
        /* 对频率进行平滑 */
        current_notch_hz = current_notch_hz * (1.0f - SMOOTH_FACTOR) + best_freq * SMOOTH_FACTOR;
        
        /* 限制范围 */
        if (current_notch_hz < 60.0f) current_notch_hz = 60.0f;
        if (current_notch_hz > 400.0f) current_notch_hz = 400.0f; // PWM 极限
        
        /* 更新滤波器 (每轴使用相同频率) */
        DRDY_IMU_SetNotchFreq(current_notch_hz, 2.5f); // Q=2.5
        
        notch_update_count++;
    }
}
