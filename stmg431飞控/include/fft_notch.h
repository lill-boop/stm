/**
 * @file fft_notch.h
 * @brief FFT 自适应双 Notch 滤波器
 * @note  使用 CMSIS-DSP arm_rfft_fast_f32
 */

#ifndef __FFT_NOTCH_H
#define __FFT_NOTCH_H

#include <stdint.h>

/* 开关宏: 0=禁用FFT Notch, 1=启用 */
#define USE_FFT_NOTCH 1

/* 调试数据 (供串口输出) */
extern float fft_peak_freq1;
extern float fft_peak_freq2;
extern float fft_peak_power1;
extern float fft_peak_power2;
extern uint32_t fft_update_count;

/**
 * @brief 初始化 FFT Notch 模块
 */
void FFTNotch_Init(void);

/**
 * @brief 执行 FFT 分析并更新 Notch
 * @note  建议 20Hz 调用
 */
void FFTNotch_Update(void);

#endif /* __FFT_NOTCH_H */
