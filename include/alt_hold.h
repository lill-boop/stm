/**
 * @file alt_hold.h
 * @brief 定高控制器 (Altitude Hold Controller)
 * @note  使用 MS5611 气压计进行高度保持
 */

#ifndef __ALT_HOLD_H
#define __ALT_HOLD_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 配置 ==================== */

/* 定高 PID 参数 (可通过 CLI 调整) */
typedef struct {
    float Kp;       // 高度误差 P (推荐 0.5-2.0)
    float Ki;       // 高度误差 I (推荐 0.0-0.1)
    float Kd;       // 垂直速度 D (推荐 0.1-0.5)
    float maxI;     // I 项限幅
} AltHoldPID_t;

/* ==================== API ==================== */

/**
 * @brief 初始化定高控制器
 */
void AltHold_Init(void);

/**
 * @brief 设置目标高度
 * @param altitude 目标高度 (m), 相对于起飞点
 */
void AltHold_SetTarget(float altitude);

/**
 * @brief 获取当前目标高度
 */
float AltHold_GetTarget(void);

/**
 * @brief 预测步骤 - 由 1kHz/2kHz ISR 调用
 * @param accZ_G 垂直加速度 (G, 已扣除重力)
 * @param dt 时间间隔 (s)
 */
void AltHold_Predict(float accZ_G, float dt);

/**
 * @brief 校正步骤 - 由 50Hz 主循环调用
 * @param baroAlt 气压计高度 (m)
 * @param dt 时间间隔 (s)
 * @return PID 输出油门修正量 (-400 ~ +400)
 */
int16_t AltHold_Correct(float baroAlt, float dt);

/**
 * @brief 获取最近一次的 PID 输出
 */
int16_t AltHold_GetOutput(void);

/**
 * @brief 更新定高控制 (兼容旧接口)
 * @deprecated 建议使用 Predict + Correct 双速率版
 */
int16_t AltHold_Update(float currentAlt, float verticalSpeed, float dt);

/**
 * @brief 重置控制器 (清除积分项等)
 */
void AltHold_Reset(void);

/**
 * @brief 启用/禁用定高模式
 */
void AltHold_Enable(bool enable);

/**
 * @brief 检查定高是否启用
 */
bool AltHold_IsEnabled(void);

/**
 * @brief 获取 PID 参数指针 (用于 CLI 调参)
 */
AltHoldPID_t* AltHold_GetPID(void);

#endif /* __ALT_HOLD_H */
