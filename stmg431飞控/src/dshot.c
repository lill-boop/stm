/**
 * @file dshot.c
 * @brief DShot600 协议驱动实现
 * @note  使用 TIM1 + DMA 实现 4 通道 DShot600
 * 
 * 引脚分配 (与原 PWM 相同):
 * - M1: PA8  (TIM1_CH1)
 * - M2: PA9  (TIM1_CH2)
 * - M3: PA10 (TIM1_CH3)
 * - M4: PB6  (TIM4_CH1) - 单独处理
 */

#include "dshot.h"
#include "motor.h"  // for USE_DSHOT macro
#include "stm32g4xx_hal.h"
#include <string.h>

/* ==================== DShot 参数 ==================== */

/* 
 * DShot600 时序 (170MHz 系统时钟):
 * 比特周期 = 1/600000 = 1.667us
 * Timer 周期 = 170MHz / 600kHz = 283.33 ≈ 284
 * 
 * 逻辑 1: 高电平 74.85% -> 284 * 0.7485 ≈ 213
 * 逻辑 0: 高电平 37.425% -> 284 * 0.3743 ≈ 106
 */
#define DSHOT_TIMER_PERIOD  283
#define DSHOT_BIT_1         213
#define DSHOT_BIT_0         106

/* DShot 帧长度: 16位数据 + 1位结束 */
#define DSHOT_FRAME_LENGTH  17

/* ==================== 私有变量 ==================== */

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim4;
static DMA_HandleTypeDef hdma_tim1_ch1;
static DMA_HandleTypeDef hdma_tim1_ch2;
static DMA_HandleTypeDef hdma_tim1_ch3;
static DMA_HandleTypeDef hdma_tim4_ch1;

/* DMA 缓冲区 (每个电机 17 个 CCR 值) */
static uint16_t dshotBuffer1[DSHOT_FRAME_LENGTH];
static uint16_t dshotBuffer2[DSHOT_FRAME_LENGTH];
static uint16_t dshotBuffer3[DSHOT_FRAME_LENGTH];
static uint16_t dshotBuffer4[DSHOT_FRAME_LENGTH];

/* 当前油门值 */
static uint16_t throttle[DSHOT_MOTOR_COUNT] = {0};

/* ==================== 私有函数 ==================== */

/**
 * @brief 计算 DShot CRC
 */
static uint8_t DShot_CalcCRC(uint16_t packet)
{
    uint8_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return crc;
}

/**
 * @brief 将油门值编码为 DShot 帧
 */
static void DShot_EncodePacket(uint16_t value, bool telemetry, uint16_t *buffer)
{
    /* 构造 16 位数据包: 11位油门 + 1位电传 + 4位CRC */
    uint16_t packet = (value << 5) | (telemetry ? 0x10 : 0x00);
    packet |= DShot_CalcCRC(packet >> 4);
    
    /* 编码为 PWM 占空比 (MSB first) */
    for (int i = 0; i < 16; i++) {
        if (packet & (1 << (15 - i))) {
            buffer[i] = DSHOT_BIT_1;
        } else {
            buffer[i] = DSHOT_BIT_0;
        }
    }
    /* 最后一位为 0，确保结束低电平 */
    buffer[16] = 0;
}

/**
 * @brief TIM1 DMA 初始化 (CH1, CH2, CH3)
 */
static void DShot_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能时钟 */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    
    /* 配置 GPIO: PA8, PA9, PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置 TIM1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;  // 不分频
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = DSHOT_TIMER_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    /* 配置 PWM 通道 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    
    /* 配置 DMA CH1 */
    hdma_tim1_ch1.Instance = DMA1_Channel1;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim1_ch1);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);
    
    /* 配置 DMA CH2 */
    hdma_tim1_ch2.Instance = DMA1_Channel2;
    hdma_tim1_ch2.Init.Request = DMA_REQUEST_TIM1_CH2;
    hdma_tim1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim1_ch2);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC2], hdma_tim1_ch2);
    
    /* 配置 DMA CH3 */
    hdma_tim1_ch3.Instance = DMA1_Channel3;
    hdma_tim1_ch3.Init.Request = DMA_REQUEST_TIM1_CH3;
    hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim1_ch3);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC3], hdma_tim1_ch3);
    
    /* DMA 中断 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    
    /* 使能 MOE (高级定时器) */
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
 * @brief TIM4 DMA 初始化 (CH1 for M4)
 */
static void DShot_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置 GPIO: PB6 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 配置 TIM4 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = DSHOT_TIMER_PERIOD;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim4);
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    
    /* 配置 DMA */
    hdma_tim4_ch1.Instance = DMA1_Channel4;
    hdma_tim4_ch1.Init.Request = DMA_REQUEST_TIM4_CH1;
    hdma_tim4_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim4_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim4_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim4_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim4_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim4_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim4_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim4_ch1);
    __HAL_LINKDMA(&htim4, hdma[TIM_DMA_ID_CC1], hdma_tim4_ch1);
    
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/* ==================== API 实现 ==================== */

/**
 * @brief DShot 初始化
 */
void DShot_Init(void)
{
    /* 清空缓冲区 */
    memset(dshotBuffer1, 0, sizeof(dshotBuffer1));
    memset(dshotBuffer2, 0, sizeof(dshotBuffer2));
    memset(dshotBuffer3, 0, sizeof(dshotBuffer3));
    memset(dshotBuffer4, 0, sizeof(dshotBuffer4));
    
    /* 初始化定时器和 DMA */
    DShot_TIM1_Init();
    DShot_TIM4_Init();
    
    /* 发送几帧 0 油门让电调初始化 */
    for (int i = 0; i < 10; i++) {
        DShot_WriteAllThrottle(0, 0, 0, 0);
        DShot_Update();
        HAL_Delay(1);
    }
}

/**
 * @brief 设置单个电机油门
 */
void DShot_WriteThrottle(uint8_t motor, uint16_t value)
{
    if (motor >= DSHOT_MOTOR_COUNT) return;
    
    /* 限制范围 */
    if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
    
    throttle[motor] = value;
}

/**
 * @brief 设置所有电机油门
 */
void DShot_WriteAllThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    DShot_WriteThrottle(DSHOT_MOTOR_1, m1);
    DShot_WriteThrottle(DSHOT_MOTOR_2, m2);
    DShot_WriteThrottle(DSHOT_MOTOR_3, m3);
    DShot_WriteThrottle(DSHOT_MOTOR_4, m4);
}

/**
 * @brief 发送 DShot 命令
 */
void DShot_SendCommand(uint8_t motor, DShot_Command_t cmd)
{
    if (motor >= DSHOT_MOTOR_COUNT) return;
    throttle[motor] = (uint16_t)cmd;
}

/**
 * @brief 更新 DShot 输出 (在主循环中调用)
 */
void DShot_Update(void)
{
    /* 编码所有电机的油门值 */
    DShot_EncodePacket(throttle[0], false, dshotBuffer1);
    DShot_EncodePacket(throttle[1], false, dshotBuffer2);
    DShot_EncodePacket(throttle[2], false, dshotBuffer3);
    DShot_EncodePacket(throttle[3], false, dshotBuffer4);
    
    /* 启动 DMA 传输 */
    HAL_DMA_Start(&hdma_tim1_ch1, (uint32_t)dshotBuffer1, 
                  (uint32_t)&htim1.Instance->CCR1, DSHOT_FRAME_LENGTH);
    HAL_DMA_Start(&hdma_tim1_ch2, (uint32_t)dshotBuffer2,
                  (uint32_t)&htim1.Instance->CCR2, DSHOT_FRAME_LENGTH);
    HAL_DMA_Start(&hdma_tim1_ch3, (uint32_t)dshotBuffer3,
                  (uint32_t)&htim1.Instance->CCR3, DSHOT_FRAME_LENGTH);
    HAL_DMA_Start(&hdma_tim4_ch1, (uint32_t)dshotBuffer4,
                  (uint32_t)&htim4.Instance->CCR1, DSHOT_FRAME_LENGTH);
    
    /* 使能 DMA 请求 */
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
    
    /* 启动定时器 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/**
 * @brief PWM 值转 DShot 值
 * @param pwm: 传统 PWM 值 (1000-2000)
 * @return DShot 油门值 (48-2047)
 */
uint16_t DShot_PWMToThrottle(uint16_t pwm)
{
    if (pwm <= 1000) return DSHOT_THROTTLE_MIN;
    if (pwm >= 2000) return DSHOT_THROTTLE_MAX;
    
    /* 线性映射: 1000->48, 2000->2047 */
    return DSHOT_THROTTLE_MIN + 
           (uint32_t)(pwm - 1000) * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) / 1000;
}

/* ==================== DMA 中断处理 ==================== */
/* 注意: 仅当 USE_DSHOT 启用时才编译，避免与 drdy_imu.c 冲突 */

#if USE_DSHOT

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim1_ch2);
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim1_ch3);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim4_ch1);
}

#endif /* USE_DSHOT */
