/**
 * @file motor.h
 * @brief 电机控制模块 (支持 PWM 和 DShot600)
 * @note  4路输出，驱动无刷电调
 * 
 * 协议选择 (在下方定义):
 * - USE_DSHOT = 0: 传统 PWM (兼容所有电调)
 * - USE_DSHOT = 1: DShot600 (需要支持 DShot 的电调)
 * 
 * PWM 规格:
 * - 频率: 400Hz
 * - 油门范围: 1000us ~ 2000us
 * 
 * DShot600 规格:
 * - 比特率: 600kbps
 * - 油门范围: 48 ~ 2047 (内部自动转换)
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* ==================== 协议选择 ==================== */
/* 设置为 1 启用 DShot600，设置为 0 使用传统 PWM */
#define USE_DSHOT   0   // TODO: 买了新电调后改为 1

/* 电机编号定义 (四旋翼X模式) */
/*
 *      机头
 *    M1 ╲╱ M2
 *      ╱╲
 *    M4    M3
 *
 * M1: 右前 (逆时针)
 * M2: 左前 (顺时针)
 * M3: 左后 (逆时针)
 * M4: 右后 (顺时针)
 */
#define MOTOR_1     0   // 右前
#define MOTOR_2     1   // 左前
#define MOTOR_3     2   // 左后
#define MOTOR_4     3   // 右后
#define MOTOR_COUNT 4

/* PWM参数 (当 USE_DSHOT = 0 时使用) */
#define MOTOR_PWM_MIN       1000    // 最小脉宽 (us)
#define MOTOR_PWM_MAX       2000    // 最大脉宽 (us)
#define MOTOR_PWM_IDLE      1050    // 怠速脉宽 (us)
#define MOTOR_PWM_FREQ      400     // PWM频率 (Hz)

/* 电机输出结构 */
typedef struct {
    uint16_t motor[MOTOR_COUNT];    // 各电机PWM值 (1000~2000)
    uint8_t  armed;                 // 解锁状态
} MotorOutput_t;

/* ==================== API函数 ==================== */

/**
 * @brief 初始化电机PWM
 * @note  使用TIM1 CH1~CH4, 引脚: PA8, PA9, PA10, PA11
 */
void Motor_Init(void);

/**
 * @brief 设置单个电机PWM
 * @param motor 电机编号 (0~3)
 * @param pwm PWM值 (1000~2000)
 */
void Motor_SetPWM(uint8_t motor, uint16_t pwm);

/**
 * @brief 设置所有电机PWM
 * @param m1,m2,m3,m4 各电机PWM值
 */
void Motor_SetAllPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/**
 * @brief 电机解锁
 */
void Motor_Arm(void);

/**
 * @brief 电机锁定 (停止所有电机)
 */
void Motor_Disarm(void);

/**
 * @brief 检查是否解锁
 */
uint8_t Motor_IsArmed(void);

/**
 * @brief 混控计算 (根据油门+姿态控制量计算各电机输出)
 * @param throttle 油门值 (0~1000)
 * @param roll     横滚控制量 (-500~500)
 * @param pitch    俯仰控制量 (-500~500)
 * @param yaw      偏航控制量 (-500~500)
 * @param output   输出的电机PWM值
 */
void Motor_Mix(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw,
               MotorOutput_t *output);

/**
 * @brief [PWM Sync] 设置待更新的PWM值 (由PID线程调用)
 * @note 值将在下个TIM中断时原子更新
 */
void Motor_SetPendingPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/**
 * @brief [PWM Sync] 获取PWM同步统计信息
 * @param updateCount 返回已更新次数
 */
void Motor_GetSyncStats(uint32_t *updateCount);

#endif /* __MOTOR_H */
