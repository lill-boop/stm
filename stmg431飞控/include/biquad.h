/**
 * @file biquad.h
 * @brief Biquad 数字滤波器 (二阶IIR)
 * 
 * 支持:
 * - 低通滤波器 (Lowpass)
 * - 陷波滤波器 (Notch)
 * 
 * 使用直接II型转置结构，数值稳定
 */

#ifndef __BIQUAD_H
#define __BIQUAD_H

#include <stdint.h>

/* 滤波器类型 */
typedef enum {
    BIQUAD_LOWPASS,
    BIQUAD_NOTCH,
    BIQUAD_HIGHPASS
} BiquadType_t;

/* Biquad 滤波器结构 */
typedef struct {
    float b0, b1, b2;   // 分子系数
    float a1, a2;       // 分母系数 (a0 = 1)
    float z1, z2;       // 延迟状态
} Biquad_t;

/* ==================== API ==================== */

/**
 * @brief 初始化低通滤波器
 * @param filter 滤波器实例
 * @param sampleRate 采样率 (Hz)
 * @param cutoffFreq 截止频率 (Hz)
 */
void Biquad_InitLowpass(Biquad_t *filter, float sampleRate, float cutoffFreq);

/**
 * @brief 初始化陷波滤波器
 * @param filter 滤波器实例
 * @param sampleRate 采样率 (Hz)
 * @param notchFreq 陷波中心频率 (Hz)
 * @param q Q因子 (典型值 3-10, 越大越尖锐)
 */
void Biquad_InitNotch(Biquad_t *filter, float sampleRate, float notchFreq, float q);

/**
 * @brief 应用滤波器
 * @param filter 滤波器实例
 * @param input 输入样本
 * @return 滤波后输出
 */
float Biquad_Apply(Biquad_t *filter, float input);

/**
 * @brief 重置滤波器状态
 */
void Biquad_Reset(Biquad_t *filter);

/**
 * @brief 更新陷波频率 (动态Notch用)
 */
void Biquad_UpdateNotchFreq(Biquad_t *filter, float sampleRate, float notchFreq, float q);

/**
 * @brief 更新低通截止频率 (动态LPF用)
 */
void Biquad_UpdateLPF(Biquad_t *filter, float sampleRate, float cutoffFreq);

#endif /* __BIQUAD_H */
