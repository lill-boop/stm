/**
 * @file fft_notch.c
 * @brief FFT 自适应双 Notch 滤波器实现
 * @note  使用内置 radix-2 FFT，无需外部库
 */

#include "fft_notch.h"
#include "drdy_imu.h"
#include <math.h>
#include <string.h>

#if USE_FFT_NOTCH

/* FFT 参数 */
#define FFT_SIZE        256
#define SAMPLE_RATE     2000.0f
#define BIN_WIDTH       (SAMPLE_RATE / FFT_SIZE)  // 7.8125 Hz/bin

/* 峰值检测范围 (bin 索引) */
#define MIN_BIN         ((int)(60.0f / BIN_WIDTH))   // ~8
#define MAX_BIN         ((int)(500.0f / BIN_WIDTH))  // ~64

/* 门限与平滑 */
#define MIN_POWER       500.0f    // 低于此能量不更新
#define FREQ_SLEW_LIMIT 20.0f     // 每次更新最大频率变化 (Hz)
#define SMOOTH_ALPHA    0.2f      // 频率平滑因子

/* 调试输出 */
float fft_peak_freq1 = 150.0f;
float fft_peak_freq2 = 300.0f;
float fft_peak_power1 = 0.0f;
float fft_peak_power2 = 0.0f;
uint32_t fft_update_count = 0;

/* FFT 缓冲 */
static float fftReal[FFT_SIZE];
static float fftImag[FFT_SIZE];
static float magSquared[FFT_SIZE / 2];

/* Hann 窗 */
static float hannWindow[FFT_SIZE];

/* 当前使用的频率 */
static float current_f1 = 150.0f;
static float current_f2 = 300.0f;

/* 位逆序表 */
static uint16_t bitrev[FFT_SIZE];

/* PI 常量 */
#define PI_F 3.14159265359f

/**
 * @brief 计算位逆序
 */
static uint16_t bit_reverse(uint16_t x, int bits)
{
    uint16_t result = 0;
    for (int i = 0; i < bits; i++) {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    return result;
}

/**
 * @brief 初始化 FFT 模块
 */
void FFTNotch_Init(void)
{
    /* 生成 Hann 窗 */
    for (int i = 0; i < FFT_SIZE; i++) {
        hannWindow[i] = 0.5f * (1.0f - cosf(2.0f * PI_F * i / (FFT_SIZE - 1)));
    }
    
    /* 预计算位逆序表 */
    int bits = 8; // log2(256) = 8
    for (int i = 0; i < FFT_SIZE; i++) {
        bitrev[i] = bit_reverse(i, bits);
    }
}

/**
 * @brief Radix-2 DIT FFT (in-place)
 */
static void fft_radix2(float *real, float *imag, int n)
{
    /* 位逆序重排 */
    for (int i = 0; i < n; i++) {
        if (bitrev[i] > i) {
            float tr = real[i];
            float ti = imag[i];
            real[i] = real[bitrev[i]];
            imag[i] = imag[bitrev[i]];
            real[bitrev[i]] = tr;
            imag[bitrev[i]] = ti;
        }
    }
    
    /* FFT 蝶形运算 */
    for (int len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * PI_F / len;
        float wpr = cosf(angle);
        float wpi = sinf(angle);
        
        for (int i = 0; i < n; i += len) {
            float wr = 1.0f, wi = 0.0f;
            int half = len >> 1;
            
            for (int j = 0; j < half; j++) {
                int a = i + j;
                int b = a + half;
                
                float tr = wr * real[b] - wi * imag[b];
                float ti = wr * imag[b] + wi * real[b];
                
                real[b] = real[a] - tr;
                imag[b] = imag[a] - ti;
                real[a] += tr;
                imag[a] += ti;
                
                /* 旋转因子更新 */
                float tmp = wr;
                wr = tmp * wpr - wi * wpi;
                wi = tmp * wpi + wi * wpr;
            }
        }
    }
}

/**
 * @brief 查找次峰 (避开主峰附近)
 */
static int FindSecondPeak(float *mag, int size, int mainPeakBin)
{
    float maxVal = 0.0f;
    int maxIdx = mainPeakBin * 2; // 默认用谐波
    
    for (int i = MIN_BIN; i < size && i <= MAX_BIN; i++) {
        if (i < mainPeakBin - 5 || i > mainPeakBin + 5) {
            if (mag[i] > maxVal) {
                maxVal = mag[i];
                maxIdx = i;
            }
        }
    }
    
    if (maxVal < MIN_POWER * 0.3f) {
        maxIdx = mainPeakBin * 2;
        if (maxIdx >= size) maxIdx = size - 1;
    }
    
    return maxIdx;
}

/**
 * @brief 执行 FFT 分析并更新 Notch
 */
void FFTNotch_Update(void)
{
    static int divider = 0;
    divider++;
    if (divider < 3) return; // 50Hz / 3 ≈ 16Hz
    divider = 0;
    
    /* 1. 获取 Gyro 数据 */
    DRDY_IMU_GetGyroBuffer(FFT_SIZE, fftReal);
    
    /* 2. 去直流 + 加 Hann 窗 */
    float sum = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) sum += fftReal[i];
    float mean = sum / FFT_SIZE;
    
    for (int i = 0; i < FFT_SIZE; i++) {
        fftReal[i] = (fftReal[i] - mean) * hannWindow[i];
        fftImag[i] = 0.0f;
    }
    
    /* 3. FFT */
    fft_radix2(fftReal, fftImag, FFT_SIZE);
    
    /* 4. 计算幅度平方 */
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        magSquared[i] = fftReal[i] * fftReal[i] + fftImag[i] * fftImag[i];
    }
    
    /* 5. 峰值检测 - 主峰 */
    float maxPower = 0.0f;
    int mainBin = MIN_BIN;
    
    for (int i = MIN_BIN; i <= MAX_BIN && i < FFT_SIZE / 2; i++) {
        if (magSquared[i] > maxPower) {
            maxPower = magSquared[i];
            mainBin = i;
        }
    }
    
    float detected_f1 = mainBin * BIN_WIDTH;
    fft_peak_power1 = maxPower;
    
    /* 6. 次峰检测 */
    int secondBin = FindSecondPeak(magSquared, FFT_SIZE / 2, mainBin);
    float detected_f2 = secondBin * BIN_WIDTH;
    fft_peak_power2 = magSquared[secondBin];
    
    /* 7. 只有能量足够才更新 */
    if (maxPower > MIN_POWER) {
        /* 频率平滑 */
        float target_f1 = current_f1 * (1.0f - SMOOTH_ALPHA) + detected_f1 * SMOOTH_ALPHA;
        float target_f2 = current_f2 * (1.0f - SMOOTH_ALPHA) + detected_f2 * SMOOTH_ALPHA;
        
        /* 跳变限幅 */
        if (target_f1 - current_f1 > FREQ_SLEW_LIMIT) target_f1 = current_f1 + FREQ_SLEW_LIMIT;
        if (current_f1 - target_f1 > FREQ_SLEW_LIMIT) target_f1 = current_f1 - FREQ_SLEW_LIMIT;
        if (target_f2 - current_f2 > FREQ_SLEW_LIMIT) target_f2 = current_f2 + FREQ_SLEW_LIMIT;
        if (current_f2 - target_f2 > FREQ_SLEW_LIMIT) target_f2 = current_f2 - FREQ_SLEW_LIMIT;
        
        /* 限制范围 */
        if (target_f1 < 60.0f) target_f1 = 60.0f;
        if (target_f1 > 400.0f) target_f1 = 400.0f;
        if (target_f2 < 60.0f) target_f2 = 60.0f;
        if (target_f2 > 500.0f) target_f2 = 500.0f;
        
        current_f1 = target_f1;
        current_f2 = target_f2;
        
        /* 更新滤波器 */
        DRDY_IMU_SetNotchFreq(current_f1, 3.0f);   // Notch1: Q=3.0
        DRDY_IMU_SetNotchFreq2(current_f2, 2.0f);  // Notch2: Q=2.0
        
        fft_update_count++;
    }
    
    /* 调试输出 */
    fft_peak_freq1 = current_f1;
    fft_peak_freq2 = current_f2;
}

#else /* USE_FFT_NOTCH == 0 */

void FFTNotch_Init(void) {}
void FFTNotch_Update(void) {}

#endif /* USE_FFT_NOTCH */
