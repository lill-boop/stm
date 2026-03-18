/**
 * @file alt_hold.c
 * @brief 定高控制器实现 (双速率融合版)
 * @note  1kHz 预测 + 50Hz 校正 = 最优响应
 * 
 * 架构:
 * - AltHold_Predict(): 由 2kHz ISR 调用,使用加速度积分预测
 * - AltHold_Correct(): 由 50Hz 主循环调用,使用气压计校正
 * - AltHold_GetOutput(): 获取 PID 输出给混控
 */

#include "alt_hold.h"
#include <math.h>

/* ==================== 私有变量 ==================== */
static AltHoldPID_t pid = {
    .Kp = 1.0f,     // 高度误差增益
    .Ki = 0.02f,    // 积分增益
    .Kd = 0.3f,     // 速度阻尼
    .maxI = 200.0f  // 积分限幅
};

static float targetAlt = 0.0f;
static float integrator = 0.0f;
static bool enabled = false;

/* ==================== 状态估计 (双速率互补滤波) ==================== */
static float estAlt = 0.0f;     // 估计高度 (cm)
static float estVel = 0.0f;     // 估计速度 (cm/s)
static int16_t lastOutput = 0;  // 缓存输出

// 滤波器增益
#define K_POS  0.02f   // 气压计位置修正权重 (50Hz 校正较少)
#define K_VEL  0.05f   // 气压计速度修正权重

/* ==================== API 实现 ==================== */

void AltHold_Init(void)
{
    targetAlt = 0.0f;
    integrator = 0.0f;
    enabled = false;
    estAlt = 0.0f;
    estVel = 0.0f;
    lastOutput = 0;
}

void AltHold_SetTarget(float altitude)
{
    targetAlt = altitude;
    integrator = 0.0f;
}

float AltHold_GetTarget(void)
{
    return targetAlt;
}

/**
 * @brief 预测步骤 - 1kHz/2kHz 调用 (ISR 内)
 * @param accZ_G 垂直加速度 (G, 已扣除重力)
 * @param dt 时间间隔 (s)
 * @note  仅做积分预测，不做 PID 计算
 */
void AltHold_Predict(float accZ_G, float dt)
{
    if (!enabled || dt <= 0.0f) return;
    
    /* 转换单位: G -> cm/s^2 */
    float accZCmSS = accZ_G * 980.0f;
    
    /* 死区 (微小震动忽略) */
    if (fabsf(accZ_G) < 0.03f) accZCmSS = 0.0f;
    
    /* 预测步骤 (加速度二次积分) */
    estAlt += estVel * dt + 0.5f * accZCmSS * dt * dt;
    estVel += accZCmSS * dt;
    
    /* 速度限幅 (防止积分发散) */
    if (estVel > 500.0f) estVel = 500.0f;   // 5m/s max
    if (estVel < -500.0f) estVel = -500.0f;
}

/**
 * @brief 校正步骤 - 50Hz 调用 (主循环)
 * @param baroAlt 气压计高度 (m)
 * @param dt 时间间隔 (s), 约 0.02s
 * @return PID 输出油门修正量 (-400 ~ +400)
 */
int16_t AltHold_Correct(float baroAlt, float dt)
{
    if (!enabled || dt <= 0.0f) {
        estAlt = baroAlt * 100.0f;
        estVel = 0.0f;
        lastOutput = 0;
        return 0;
    }
    
    /* 1. 气压计校正 (互补滤波) */
    float baroAltCm = baroAlt * 100.0f;
    float altError = baroAltCm - estAlt;
    
    estAlt += K_POS * altError;
    estVel += K_VEL * altError / dt;  // 速度校正
    
    /* 2. PID 控制 */
    float targetAltCm = targetAlt * 100.0f;
    float posError = targetAltCm - estAlt;
    
    /* P 项 */
    float pTerm = pid.Kp * posError;
    
    /* I 项 (带抗饱和) */
    integrator += pid.Ki * posError * dt;
    if (integrator > pid.maxI) integrator = pid.maxI;
    if (integrator < -pid.maxI) integrator = -pid.maxI;
    float iTerm = integrator;
    
    /* D 项 (速度环) */
    float targetSpeed = posError * 1.5f;
    if (targetSpeed > 150.0f) targetSpeed = 150.0f;
    if (targetSpeed < -100.0f) targetSpeed = -100.0f;
    
    float speedError = targetSpeed - estVel;
    float dTerm = pid.Kd * speedError;
    
    /* 3. 总输出 */
    float output = pTerm + iTerm + dTerm;
    if (output > 400.0f) output = 400.0f;
    if (output < -400.0f) output = -400.0f;
    
    lastOutput = (int16_t)output;
    return lastOutput;
}

/**
 * @brief 兼容旧接口 (单次调用版)
 * @deprecated 建议使用 Predict + Correct 双速率版
 */
int16_t AltHold_Update(float currentAltBaro, float accZ_G, float dt)
{
    AltHold_Predict(accZ_G, dt);
    return AltHold_Correct(currentAltBaro, dt);
}

/**
 * @brief 获取最近一次的输出 (可在 1kHz 内使用)
 */
int16_t AltHold_GetOutput(void)
{
    return lastOutput;
}

void AltHold_Reset(void)
{
    integrator = 0.0f;
    estVel = 0.0f;
    lastOutput = 0;
}

void AltHold_Enable(bool enable)
{
    if (enable && !enabled) {
        integrator = 0.0f;
    }
    enabled = enable;
}

bool AltHold_IsEnabled(void)
{
    return enabled;
}

AltHoldPID_t* AltHold_GetPID(void)
{
    return &pid;
}
