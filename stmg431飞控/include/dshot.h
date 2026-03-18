/**
 * @file dshot.h
 * @brief DShot600 协议驱动
 * @note  使用 DMA + Timer 实现高效数字电调通信
 * 
 * DShot600 协议说明:
 * - 比特率: 600,000 bit/s
 * - 帧格式: 16位 (11位油门 + 1位电传请求 + 4位CRC)
 * - 电平: 高电平占空比 74.85% = 1, 37.425% = 0
 */

#ifndef __DSHOT_H
#define __DSHOT_H

#include <stdint.h>
#include <stdbool.h>

/* DShot 类型 */
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQUEST = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_MAX = 47
} DShot_Command_t;

/* 电机编号 */
#define DSHOT_MOTOR_1   0
#define DSHOT_MOTOR_2   1
#define DSHOT_MOTOR_3   2
#define DSHOT_MOTOR_4   3
#define DSHOT_MOTOR_COUNT 4

/* 油门范围 */
#define DSHOT_THROTTLE_MIN  48      // 最小油门 (0-47 是命令)
#define DSHOT_THROTTLE_MAX  2047    // 最大油门 (11位)

/* API */
void DShot_Init(void);
void DShot_WriteThrottle(uint8_t motor, uint16_t throttle);
void DShot_WriteAllThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void DShot_SendCommand(uint8_t motor, DShot_Command_t cmd);
void DShot_Update(void);  // 在主循环中调用，触发 DMA 发送

/* 转换函数: PWM值 (1000-2000) -> DShot值 (48-2047) */
uint16_t DShot_PWMToThrottle(uint16_t pwm);

#endif /* __DSHOT_H */
