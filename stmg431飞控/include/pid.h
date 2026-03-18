/**
 * @file pid.h
 * @brief PID控制器模块
 * @note  飞控使用串级PID: 外环(角度) + 内环(角速度)
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/* PID控制器结构体 */
typedef struct {
    /* 参数 */
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    /* 限幅 */
    float outputMax;    // 输出上限
    float outputMin;    // 输出下限
    float integralMax;  // 积分限幅
    
    /* 内部状态 */
    float integral;     // 积分累积值
    float lastError;    // 上次误差
    float lastDerivative; // 上次微分 (用于低通滤波)
    float dLpfAlpha;    // D项低通滤波系数 (动态可调)
    
    /* 输出 */
    float output;       // 当前输出
} PID_t;

/* 飞控PID组 */
typedef struct {
    PID_t roll_rate;    // 横滚角速度环
    PID_t pitch_rate;   // 俯仰角速度环
    PID_t yaw_rate;     // 偏航角速度环
    
    PID_t roll_angle;   // 横滚角度环
    PID_t pitch_angle;  // 俯仰角度环
} FlightPID_t;

/* ==================== API函数 ==================== */

/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限幅
 */
void PID_SetOutputLimits(PID_t *pid, float min, float max);

/**
 * @brief 设置积分限幅
 */
void PID_SetIntegralLimit(PID_t *pid, float limit);

/**
 * @brief PID计算 (带微分滤波)
 * @param pid PID控制器
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param dt 时间间隔 (秒)
 * @return 控制输出
 */
float PID_Calculate(PID_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 重置PID状态
 */
void PID_Reset(PID_t *pid);
void PID_ResetIntegral(PID_t *pid);

/**
 * @brief 初始化飞控PID (使用默认参数)
 */
void FlightPID_Init(FlightPID_t *fpid);

#endif /* __PID_H */
