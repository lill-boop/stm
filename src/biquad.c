/**
 * @file biquad.c
 * @brief Biquad 数字滤波器实现
 * @note  使用直接II型转置结构
 */

#include "biquad.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化低通滤波器 (Butterworth)
 */
void Biquad_InitLowpass(Biquad_t *filter, float sampleRate, float cutoffFreq)
{
    /* 预计算 */
    float omega = 2.0f * M_PI * cutoffFreq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707107f);  // Q = sqrt(2)/2 for Butterworth
    
    /* 计算系数 */
    float a0 = 1.0f + alpha;
    
    filter->b0 = (1.0f - cs) / 2.0f / a0;
    filter->b1 = (1.0f - cs) / a0;
    filter->b2 = (1.0f - cs) / 2.0f / a0;
    filter->a1 = -2.0f * cs / a0;
    filter->a2 = (1.0f - alpha) / a0;
    
    /* 清零状态 */
    filter->z1 = 0;
    filter->z2 = 0;
}

/**
 * @brief 初始化陷波滤波器
 */
void Biquad_InitNotch(Biquad_t *filter, float sampleRate, float notchFreq, float q)
{
    Biquad_UpdateNotchFreq(filter, sampleRate, notchFreq, q);
    filter->z1 = 0;
    filter->z2 = 0;
}

/**
 * @brief 更新陷波频率 (可动态调用)
 */
void Biquad_UpdateNotchFreq(Biquad_t *filter, float sampleRate, float notchFreq, float q)
{
    /* 频率限制 */
    if (notchFreq < 10.0f) notchFreq = 10.0f;
    if (notchFreq > sampleRate * 0.48f) notchFreq = sampleRate * 0.48f;
    
    float omega = 2.0f * M_PI * notchFreq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q);
    
    float a0 = 1.0f + alpha;
    
    filter->b0 = 1.0f / a0;
    filter->b1 = -2.0f * cs / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = -2.0f * cs / a0;
    filter->a2 = (1.0f - alpha) / a0;
}

/**
 * @brief 应用滤波器 (直接II型转置)
 */
float Biquad_Apply(Biquad_t *filter, float input)
{
    float output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}

/**
 * @brief 动态更新低通截止频率 (不重置state)
 */
void Biquad_UpdateLPF(Biquad_t *filter, float sampleRate, float cutoffFreq)
{
    /* 频率限制 */
    if (cutoffFreq < 10.0f) cutoffFreq = 10.0f;
    if (cutoffFreq > sampleRate * 0.48f) cutoffFreq = sampleRate * 0.48f;
    
    /* 重新计算系数 (与InitLowpass相同逻辑) */
    float omega = 2.0f * M_PI * cutoffFreq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707107f);  // Q = sqrt(2)/2 for Butterworth
    
    float a0 = 1.0f + alpha;
    
    filter->b0 = (1.0f - cs) / 2.0f / a0;
    filter->b1 = (1.0f - cs) / a0;
    filter->b2 = (1.0f - cs) / 2.0f / a0;
    filter->a1 = -2.0f * cs / a0;
    filter->a2 = (1.0f - alpha) / a0;
    
    /* 注意：不重置 z1, z2，保持滤波器状态连续 */
}

/**
 * @brief 重置滤波器状态
 */
void Biquad_Reset(Biquad_t *filter)
{
    filter->z1 = 0;
    filter->z2 = 0;
}
