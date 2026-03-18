/**
 * @file pid.c
 * @brief PID控制器实现
 * @note  带积分限幅、输出限幅、微分滤波
 */

#include "pid.h"
#include <math.h>

/* 低通滤波系数 (微分项滤波) */
/* 低通滤波系数 (微分项滤波, 越小滤波越强) */
/* F450机架震动大，建议 0.1~0.2 */
#define PID_D_FILTER_COEFF  0.15f

/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    /* 默认限幅 */
    pid->outputMax = 500.0f;
    pid->outputMin = -500.0f;
    pid->integralMax = 200.0f;
    
    /* 清零状态 */
    pid->integral = 0.0f;
    pid->lastError = 0.0f;
    pid->lastDerivative = 0.0f;
    pid->output = 0.0f;
    
    /* 默认 D-term LPF = 60Hz @ 2kHz (dt=0.0005) -> alpha = 60 * 6.28 * 0.0005 = 0.188 */
    pid->dLpfAlpha = 0.188f;
}

/**
 * @brief 设置输出限幅
 */
void PID_SetOutputLimits(PID_t *pid, float min, float max)
{
    pid->outputMin = min;
    pid->outputMax = max;
}

/**
 * @brief 设置积分限幅
 */
void PID_SetIntegralLimit(PID_t *pid, float limit)
{
    pid->integralMax = limit;
}

/**
 * @brief 重置PID状态
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->lastError = 0.0f;
    pid->lastDerivative = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 重置PID积分项 (防疯转关键!)
 */
void PID_ResetIntegral(PID_t *pid)
{
    pid->integral = 0.0f;
}

/**
 * @brief PID计算 (增强版)
 * @note  包含: 抗饱和, I-term Relax, D-on-Measurement
 */
float PID_Calculate(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    float pTerm, iTerm, dTerm;
    float derivative;
    
    /* P项 */
    pTerm = pid->Kp * error;
    
    /* ==================== D-on-Measurement ==================== */
    /* 对测量值求导而不是对误差求导，避免 setpoint 突变导致 D 冲击 */
    if (dt > 0.0f) {
        /* 注意：这里用 -measurement，因为 measurement 增加意味着误差减小 */
        derivative = -(measurement - pid->lastError) / dt;  // lastError 存的是 lastMeasurement
        
        /* 一阶低通滤波，减少噪声 */
        derivative = pid->lastDerivative + 
                     pid->dLpfAlpha * (derivative - pid->lastDerivative);
        pid->lastDerivative = derivative;
    } else {
        derivative = 0.0f;
    }
    dTerm = pid->Kd * derivative;
    
    /* 预计算输出 (用于抗饱和判断) */
    float preOutput = pTerm + (pid->Ki * pid->integral) + dTerm;
    
    /* ==================== I-term Relax ==================== */
    /* 当 setpoint 变化很快时，减少积分累积 (防止甩杆后回弹) */
    float setpointDelta = setpoint - pid->lastError;  // 复用存储
    float relaxFactor = 1.0f;
    
    #define ITERM_RELAX_THRESHOLD  50.0f   // 开始放松的阈值
    #define ITERM_RELAX_STRENGTH   0.02f   // 放松强度
    
    if (fabsf(setpointDelta) > ITERM_RELAX_THRESHOLD * dt) {
        /* setpoint 变化快，降低积分累积 */
        relaxFactor = 1.0f / (1.0f + fabsf(setpointDelta) * ITERM_RELAX_STRENGTH);
    }
    
    /* I项 - 带抗饱和 + I-term Relax */
    uint8_t saturated = 0;
    if (preOutput >= pid->outputMax && error > 0) {
        saturated = 1;
    } else if (preOutput <= pid->outputMin && error < 0) {
        saturated = 1;
    }
    
    if (!saturated) {
        pid->integral += error * dt * relaxFactor;  // 应用 Relax 因子
        /* 积分限幅 */
        if (pid->integral > pid->integralMax) {
            pid->integral = pid->integralMax;
        } else if (pid->integral < -pid->integralMax) {
            pid->integral = -pid->integralMax;
        }
    }
    iTerm = pid->Ki * pid->integral;
    
    /* 保存当前测量值 (D-on-Measurement 需要) */
    pid->lastError = measurement;  // 注意：这里存的是 measurement，用于 D 计算
    
    /* 计算最终输出 */
    pid->output = pTerm + iTerm + dTerm;
    
    /* 输出限幅 */
    if (pid->output > pid->outputMax) {
        pid->output = pid->outputMax;
    } else if (pid->output < pid->outputMin) {
        pid->output = pid->outputMin;
    }
    
    return pid->output;
}

/**
 * @brief 初始化飞控PID (经验参数，需要调试)
 */
void FlightPID_Init(FlightPID_t *fpid)
{
    /* 角速度环 (内环) - F450机架调优参数 */
    PID_Init(&fpid->roll_rate,   6.0f, 0.05f, 0.3f);  // P加硬，I抗风，D适当降低
    PID_Init(&fpid->pitch_rate,  6.0f, 0.05f, 0.3f);
    PID_Init(&fpid->yaw_rate,    8.0f, 0.05f, 0.0f);  // Yaw需要很大P才能锁住
    
    /* 角度环 (外环) - 决定稳定性 */
    PID_Init(&fpid->roll_angle,  6.0f, 0.0f, 0.0f);   // 响应更快
    PID_Init(&fpid->pitch_angle, 6.0f, 0.0f, 0.0f);
    
    /* 设置限幅 */
    PID_SetOutputLimits(&fpid->roll_rate,   -500.0f, 500.0f);
    PID_SetOutputLimits(&fpid->pitch_rate,  -500.0f, 500.0f);
    PID_SetOutputLimits(&fpid->yaw_rate,    -300.0f, 300.0f);
    PID_SetOutputLimits(&fpid->roll_angle,  -300.0f, 300.0f);  // 输出是角速度目标
    PID_SetOutputLimits(&fpid->pitch_angle, -300.0f, 300.0f);
}
