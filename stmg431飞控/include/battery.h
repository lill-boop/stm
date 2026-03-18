/**
 * @file battery.h
 * @brief 电池电压检测模块
 * @note  使用ADC读取分压后的电池电压
 * 
 * 电路设计:
 * Battery (+) --[R1]--+--[R2]-- GND
 *                     |
 *               STM32 (PA1)
 * 
 * 推荐: R1=10k, R2=1k (分压比 11:1), 最大可测 3.3V * 11 = 36.3V
 */

#ifndef __BATTERY_H
#define __BATTERY_H

#include <stdint.h>

/* 参数配置 */
#define BATTERY_ADC_PIN     GPIO_PIN_1
#define BATTERY_ADC_PORT    GPIOA
#define BATTERY_DIVIDER_R1  10000.0f  // 上拉电阻 10k
#define BATTERY_DIVIDER_R2  1000.0f   // 下拉电阻 1k
#define ADC_REF_VOLTAGE     3.3f      // 参考电压

/* 警报阈值 (3S锂电池) */
#define BATTERY_WARN_VOLT   10.8f     // 报警电压 (3.6V/cell)
#define BATTERY_CRIT_VOLT   10.5f     // 强制降落电压 (3.5V/cell)

/* ==================== API函数 ==================== */

/**
 * @brief 初始化电池检测 (ADC1 IN2 -> PA1)
 */
void Battery_Init(void);

/**
 * @brief 读取电池电压 (伏特)
 */
float Battery_GetVoltage(void);

/**
 * @brief 检查是否低压报警
 */
uint8_t Battery_IsLowVoltage(void);

#endif /* __BATTERY_H */
