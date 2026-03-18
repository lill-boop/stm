/**
 * @file attitude.c
 * @brief Madgwick姿态解算算法实现
 * @note  基于Sebastian Madgwick的开源AHRS算法
 *        参考: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * 
 * 算法特点:
 * - 计算量小，适合嵌入式
 * - 不需要磁力计也能工作（6轴模式）
 * - 自动补偿陀螺仪漂移
 */

#include "attitude.h"
#include <math.h>

/* ==================== 私有变量 ==================== */

/* 四元数 (初始为单位四元数，表示水平姿态) */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* 滤波器参数 */
static float beta = 0.1f;           // 收敛速度
static float samplePeriod = 0.001f; // 采样周期

/* [Optimal-D] 欧拉角缓存 (500Hz 计算，2kHz 使用) */
static Attitude_t cached_euler = {0.0f, 0.0f, 0.0f};

/* 辅助函数 */
static float invSqrt(float x);

/* ==================== API实现 ==================== */

/**
 * @brief 初始化姿态解算
 */
void Attitude_Init(float sampleFreq)
{
    samplePeriod = 1.0f / sampleFreq;
    
    /* 重置四元数为单位四元数 */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

/**
 * @brief 设置Beta参数
 */
void Attitude_SetBeta(float b)
{
    beta = b;
}

/**
 * @brief Madgwick 6轴IMU更新算法
 * @param gx,gy,gz 陀螺仪 (rad/s)
 * @param ax,ay,az 加速度 (任意单位，会归一化)
 */
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
    float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* 四元数微分方程（陀螺仪积分） */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    /* 如果加速度有效，则用加速度修正陀螺仪漂移 */
    /* 鲁棒估计器: Accel Gating */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        /* 计算加速度模长 (单位: g) */
        /* 注意：这里需要先计算模长用于门控，再归一化用于解算 */
        float normSq = ax * ax + ay * ay + az * az;
        recipNorm = invSqrt(normSq);
        
        /* Accel Gating: 信任度检查 */
        /* 理想情况下 normSq 应接近 1.0 (1g^2) */
        /* 如果偏差过大 (震动或大机动)，降低 Beta 增益 */
        /* Accel Gating: 信任度检查 */
        /* 理想情况下 normSq 应接近 1.0 (1g^2) */
        float norm = normSq * recipNorm; // = sqrt(normSq)
        float error = fabsf(norm - 1.0f);
        float gateWeight = 1.0f;
        
        /* 引入外部 armed 状态 (需要在 attitude.h 或通过函数传入，这里简化处理假定外部定义) */
        /* 实际上 attitude.c 应该是独立的。我们通过 SetBeta 接口或改用纯数学逻辑 */
        /* 策略: 如果 error 很小，信任；如果 error 大，不信 */
        /* 但是：在上电静止校准阶段(未解锁)，应该无视小震动强制收敛 */
        /* 由于 attitude.c 不知道 armed 状态，我们这里只做纯物理 gating */
        /* 解锁状态下的强制收敛由 main.c 设置高 beta 来实现 */
        
        /* 线性权重: 0.1g 开始衰减，0.4g 完全截止 */
        if (error > 0.4f) {
            gateWeight = 0.0f;
        } else if (error > 0.1f) {
            /* Linear: 1.0 -> 0.0 over 0.1 -> 0.4 */
            /* w = 1.0 - (err - 0.1) / (0.3) */
            gateWeight = 1.0f - (error - 0.1f) / 0.3f;
        }
        
        /* 应用权重到 Beta */
        float effectiveBeta = beta * gateWeight;
        
        /* 只有当信任度足够时才应用修正 */
        if (effectiveBeta > 0.0001f) {
             /* 归一化加速度 */
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            /* 预计算常用值 */
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            /* 梯度下降法计算误差 */
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            
            /* 归一化梯度 */
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* 应用误差修正 (使用门控后的 beta) */
            qDot1 -= effectiveBeta * s0;
            qDot2 -= effectiveBeta * s1;
            qDot3 -= effectiveBeta * s2;
            qDot4 -= effectiveBeta * s3;
        }
    }

    /* 积分得到新的四元数 */
    q0 += qDot1 * samplePeriod;
    q1 += qDot2 * samplePeriod;
    q2 += qDot3 * samplePeriod;
    q3 += qDot4 * samplePeriod;

    /* 归一化四元数 */
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

/**
 * @brief 获取欧拉角 (四元数转欧拉角)
 */
void Attitude_GetEuler(Attitude_t *att)
{
    /* Roll (X轴旋转) */
    att->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    /* Pitch (Y轴旋转) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        att->pitch = copysignf(3.14159265f / 2.0f, sinp);  // 接近±90度
    } else {
        att->pitch = asinf(sinp);
    }
    
    /* Yaw (Z轴旋转) */
    att->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
    
    /* 弧度转角度 */
    att->roll  *= 57.29578f;  // 180/PI
    att->pitch *= 57.29578f;
    att->yaw   *= 57.29578f;
    
    /* Yaw范围调整到 0~360 */
    if (att->yaw < 0) {
        att->yaw += 360.0f;
    }
}

/**
 * @brief [Optimal-D] 计算欧拉角并缓存 (500Hz 调用)
 * @note 执行 atan2f/asinf 计算
 */
void Attitude_ComputeEuler(void)
{
    /* Roll */
    cached_euler.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 
                                1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    /* Pitch */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        cached_euler.pitch = copysignf(3.14159265f / 2.0f, sinp);
    } else {
        cached_euler.pitch = asinf(sinp);
    }
    
    /* Yaw */
    cached_euler.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                               1.0f - 2.0f * (q2 * q2 + q3 * q3));
    
    /* 弧度转角度 */
    cached_euler.roll  *= 57.29578f;
    cached_euler.pitch *= 57.29578f;
    cached_euler.yaw   *= 57.29578f;
    
    if (cached_euler.yaw < 0) {
        cached_euler.yaw += 360.0f;
    }
}

/**
 * @brief [Optimal-D] 获取缓存的欧拉角 (无计算)
 */
void Attitude_GetEulerCached(Attitude_t *att)
{
    *att = cached_euler;
}

/**
 * @brief 获取四元数
 */
void Attitude_GetQuaternion(Quaternion_t *q)
{
    q->q0 = q0;
    q->q1 = q1;
    q->q2 = q2;
    q->q3 = q3;
}

/**
 * @brief 获取垂直方向(Earth Z)的运动加速度 (单位: G)
 * @note  需要传入原始加速度 (未归一化)
 *        返回值为 + 表示向上加速，- 表示向下加速 (已扣除 1g 重力)
 */
float Attitude_GetEarthG(float ax, float ay, float az)
{
    /* 
     * 将机体坐标系加速度 (Body Frame) 投影到 地理坐标系 Z轴 (Earth Frame)
     * R_31 = 2(q1q3 - q0q2)
     * R_32 = 2(q0q1 + q2q3)
     * R_33 = q0^2 - q1^2 - q2^2 + q3^2
     * 
     * Acc_Earth_Z = R_31*ax + R_32*ay + R_33*az
     */
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float r31 = 2.0f * (q1q3 - q0q2);
    float r32 = 2.0f * (q0q1 + q2q3);
    float r33 = q0q0 - q1q1 - q2q2 + q3q3;

    /* 投影到 Earth Z */
    float accZ_Earth_Raw = r31 * ax + r32 * ay + r33 * az;
    
    /* 归一化单位 G */
    /* 我们假设 1g = 1000 LSB (根据日志 AZ=1001) */
    /* 必须扣除重力 1g */
    float accZ_G = (accZ_Earth_Raw / 1000.0f) - 1.0f;
    
    return accZ_G;
}

/* ==================== 辅助函数 ==================== */

/**
 * @brief 快速平方根倒数 (经典Quake3算法)
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));  // 牛顿迭代一次
    y = y * (1.5f - (halfx * y * y));  // 再迭代一次提高精度
    return y;
}
