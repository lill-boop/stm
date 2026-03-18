/**
 * @file blackbox.c
 * @brief 最小 Blackbox 日志实现
 * @note  100Hz UART DMA 二进制输出
 */

#include "blackbox.h"
#include "bsp_uart.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* ==================== 私有变量 ==================== */

static BlackboxFrame_t frame;
static uint8_t divider = 0;
static bool enabled = false;

/* 分频计数 (2kHz -> 100Hz = 每20次) */
#define BLACKBOX_DIVIDER  20

/* ==================== API 实现 ==================== */

void Blackbox_Init(void)
{
    memset(&frame, 0, sizeof(frame));
    frame.header[0] = BLACKBOX_HEADER_1;
    frame.header[1] = BLACKBOX_HEADER_2;
    divider = 0;
    enabled = false;
}

void Blackbox_Enable(bool enable)
{
    enabled = enable;
    if (enable) {
        divider = 0;
    }
}

bool Blackbox_IsEnabled(void)
{
    return enabled;
}

void Blackbox_Log(float *gyro_raw, float *gyro_filt, float *dterm, 
                  uint16_t *motor, uint16_t throttle, uint32_t dt_us)
{
    if (!enabled) return;
    
    /* 分频到 100Hz */
    if (++divider < BLACKBOX_DIVIDER) return;
    divider = 0;
    
    /* 填充帧数据 */
    frame.timestamp = (uint16_t)(HAL_GetTick() & 0xFFFF);
    
    /* 陀螺仪 (0.1 deg/s 单位) */
    frame.gyro_raw[0] = (int16_t)(gyro_raw[0] * 10.0f);
    frame.gyro_raw[1] = (int16_t)(gyro_raw[1] * 10.0f);
    frame.gyro_raw[2] = (int16_t)(gyro_raw[2] * 10.0f);
    
    frame.gyro_filt[0] = (int16_t)(gyro_filt[0] * 10.0f);
    frame.gyro_filt[1] = (int16_t)(gyro_filt[1] * 10.0f);
    frame.gyro_filt[2] = (int16_t)(gyro_filt[2] * 10.0f);
    
    /* D 项 */
    frame.dterm[0] = (int16_t)(dterm[0] * 10.0f);
    frame.dterm[1] = (int16_t)(dterm[1] * 10.0f);
    frame.dterm[2] = (int16_t)(dterm[2] * 10.0f);
    
    /* 电机 */
    frame.motor[0] = motor[0];
    frame.motor[1] = motor[1];
    frame.motor[2] = motor[2];
    frame.motor[3] = motor[3];
    
    /* 油门和 dt */
    frame.throttle = throttle;
    frame.loop_dt = (uint8_t)(dt_us / 10);  // 10us 单位
    
    /* 计算校验和 (XOR) */
    uint8_t *p = (uint8_t*)&frame;
    uint8_t checksum = 0;
    for (int i = 2; i < BLACKBOX_FRAME_SIZE - 1; i++) {
        checksum ^= p[i];
    }
    frame.checksum = checksum;
    
    /* 发送 (非阻塞，直接写) */
    /* 注意: 这里用简单的轮询发送，避免 DMA 复杂度 */
    extern UART_HandleTypeDef huart2;
    HAL_UART_Transmit(&huart2, (uint8_t*)&frame, BLACKBOX_FRAME_SIZE, 2);
}
