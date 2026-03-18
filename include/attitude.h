/**
 * @file attitude.h
 * @brief 姿态解算模块 - 基于Madgwick滤波器
 * @note  开源算法，参考: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>

/* 欧拉角结构体 (单位: 度) */
typedef struct {
    float roll;     // 横滚角 (-180 ~ +180)
    float pitch;    // 俯仰角 (-90 ~ +90)
    float yaw;      // 偏航角 (0 ~ 360)
} Attitude_t;

/* 四元数结构体 */
typedef struct {
    float q0, q1, q2, q3;
} Quaternion_t;

/* ==================== API函数 ==================== */

/**
 * @brief 初始化姿态解算
 * @param sampleFreq 采样频率 (Hz)
 */
void Attitude_Init(float sampleFreq);

/**
 * @brief 更新姿态 (6轴IMU，无磁力计)
 * @param gx,gy,gz 陀螺仪数据 (rad/s)
 * @param ax,ay,az 加速度数据 (g 或 归一化)
 */
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az);

/**
 * @brief 获取当前欧拉角
 * @param att 输出的欧拉角
 */
void Attitude_GetEuler(Attitude_t *att);

/**
 * @brief [Optimal-D] 计算欧拉角 (500Hz 调用)
 * @note 执行 atan2f/asinf 计算并缓存
 */
void Attitude_ComputeEuler(void);

/**
 * @brief [Optimal-D] 获取缓存的欧拉角 (无计算)
 * @param att 输出的欧拉角
 */
void Attitude_GetEulerCached(Attitude_t *att);

/**
 * @brief 获取当前四元数
 * @param q 输出的四元数
 */
void Attitude_GetQuaternion(Quaternion_t *q);
float Attitude_GetEarthG(float ax, float ay, float az); // 新增: 获取垂直加速度att);

/**
 * @brief 设置滤波器参数Beta
 * @param beta 收敛速度 (默认0.1)
 */
void Attitude_SetBeta(float beta);

#endif /* __ATTITUDE_H */
