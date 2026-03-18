/**
 * @file buzzer.h
 * @brief 蜂鸣器驱动
 * @note  使用有源蜂鸣器
 *        正极 -> 5V
 *        负极 -> S8050 集电极 -> 发射极接地
 *        基极 -> 1k电阻 -> PC14
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

/* Beeper Pin: PC14 */
#define BUZZER_PIN      GPIO_PIN_14
#define BUZZER_PORT     GPIOC

/* ==================== API ==================== */

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

/**
 * @brief 鸣叫特定次数
 * @note  阻塞式延时，仅用于初始化或错误
 */
void Buzzer_Beep(uint8_t count, uint32_t period);

#endif /* __BUZZER_H */
