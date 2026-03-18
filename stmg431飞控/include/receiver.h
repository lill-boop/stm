/**
 * @file receiver.h
 * @brief 遥控器接收机驱动 - PPM信号解析
 * @note  支持富斯FS-iA6/iA6B接收机的PPM输出
 * 
 * PPM信号规格:
 * - 帧周期: ~22.5ms (约44Hz)
 * - 同步脉冲: >3000us
 * - 通道脉宽: 1000us ~ 2000us (中位1500us)
 * - 通道数: 6-8个
 */

#ifndef __RECEIVER_H
#define __RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

/* 通道定义 */
#define RC_CH_COUNT     6       // 通道数量

/* 通道索引 (根据富斯i6默认设置) */
#define RC_CH_ROLL      0       // CH1 - 横滚 (右摇杆左右)
#define RC_CH_PITCH     1       // CH2 - 俯仰 (右摇杆上下)
#define RC_CH_THROTTLE  2       // CH3 - 油门 (左摇杆上下)
#define RC_CH_YAW       3       // CH4 - 偏航 (左摇杆左右)
#define RC_CH_AUX1      4       // CH5 - 辅助1 (开关SWA)
#define RC_CH_AUX2      5       // CH6 - 辅助2 (开关SWB/旋钮)

/* 脉宽范围 */
#define RC_PWM_MIN      1000    // 最小脉宽 (us)
#define RC_PWM_MAX      2000    // 最大脉宽 (us)
#define RC_PWM_MID      1500    // 中位脉宽 (us)
#define RC_PWM_DEADZONE 50      // 死区 (us)

/* 信号超时 */
#define RC_TIMEOUT_MS   500     // 信号丢失超时 (ms)

/* 接收机状态 */
typedef struct {
    uint16_t channel[RC_CH_COUNT];  // 各通道原始值 (1000~2000)
    bool     connected;             // 连接状态
    uint32_t lastUpdateTime;        // 上次更新时间
    uint8_t  frameCount;            // 帧计数
} Receiver_t;

/* ==================== API函数 ==================== */

/**
 * @brief 初始化PPM接收
 * @note  使用TIM2 CH1, 引脚PA0
 */
void Receiver_Init(void);

/**
 * @brief 更新接收机状态 (需要在主循环中调用)
 */
void Receiver_Update(void);

/**
 * @brief 获取通道值
 * @param ch 通道号 (0~5)
 * @return 脉宽值 (1000~2000), 失联返回中位值
 */
uint16_t Receiver_GetChannel(uint8_t ch);

/**
 * @brief 获取遥控器连接状态
 */
bool Receiver_IsConnected(void);

/**
 * @brief 获取油门值 (0~1000)
 */
int16_t Receiver_GetThrottle(void);

/**
 * @brief 获取横滚控制量 (-500~500)
 */
int16_t Receiver_GetRoll(void);

/**
 * @brief 获取俯仰控制量 (-500~500)
 */
int16_t Receiver_GetPitch(void);

/**
 * @brief 获取偏航控制量 (-500~500)
 */
int16_t Receiver_GetYaw(void);

/**
 * @brief 检查是否请求解锁 (油门最低+偏航最右)
 */
bool Receiver_IsArmRequest(void);

/**
 * @brief 检查是否请求锁定 (油门最低+偏航最左)
 */
bool Receiver_IsDisarmRequest(void);

#endif /* __RECEIVER_H */
