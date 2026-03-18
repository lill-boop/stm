/**
 * @file motor.c
 * @brief 电机PWM驱动实现
 * @note  使用TIM1输出4路PWM，驱动无刷电调
 * 
 * 引脚分配:
 * - TIM1_CH1 -> PA8  -> 电机1 (右前)
 * - TIM1_CH2 -> PA9  -> 电机2 (左前)
 * - TIM1_CH3 -> PA10 -> 电机3 (左后)
 * - TIM1_CH4 -> PA11 -> 电机4 (右后)
 */

#include "motor.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ==================== 私有变量 ==================== */

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim4;
static uint8_t motorArmed = 0;

/* [Phase-3] 推力线性化查表 (0-1000) */
static uint16_t thrustLinearTable[1001];

/* [PWM Sync] 双缓冲机制 */
static volatile uint16_t g_pendingPWM[4] = {1000, 1000, 1000, 1000};  // PID写入
static volatile uint8_t g_pwmReady = 0;  // 标志：新值ready
static uint32_t g_pwmUpdateCount = 0;     // 统计：已更新次数
static uint8_t g_pwmDivider = 0;          // 2kHz → 400Hz分频器

/* ==================== 私有函数 ==================== */

static void TIM1_PWM_Init(void);
static void TIM4_PWM_Init(void);

/* ==================== API实现 ==================== */

/**
 * @brief 初始化电机PWM
 */
void Motor_Init(void)
{
    TIM1_PWM_Init();
    TIM4_PWM_Init();  // M4 使用 TIM4 (PB6)
    
    /* [Phase-3] 初始化推力线性化表 */
    /* table[i] = sqrt(i/1000) * 1000 */
    for(int i=0; i<=1000; i++) {
        thrustLinearTable[i] = (uint16_t)(sqrtf((float)i / 1000.0f) * 1000.0f);
    }
    
    /* 初始化为最小油门 */
    Motor_SetAllPWM(MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN);
    
    motorArmed = 0;
}

/**
 * @brief 设置单个电机PWM
 */
void Motor_SetPWM(uint8_t motor, uint16_t pwm)
{
    /* 限制PWM范围 */
    if (pwm < MOTOR_PWM_MIN) pwm = MOTOR_PWM_MIN;
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    
    /* 未解锁时强制最小油门 */
    if (!motorArmed) {
        pwm = MOTOR_PWM_MIN;
    }
    
    switch (motor) {
        case MOTOR_1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            break;
        case MOTOR_2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            break;
        case MOTOR_3:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
            break;
        case MOTOR_4:
            // 使用 TIM4 CH1 (PB6) - PA11 有 USB 冲突
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
            break;
    }
}

/**
 * @brief 设置所有电机PWM
 */
void Motor_SetAllPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    Motor_SetPWM(MOTOR_1, m1);
    Motor_SetPWM(MOTOR_2, m2);
    Motor_SetPWM(MOTOR_3, m3);
    Motor_SetPWM(MOTOR_4, m4);
}

/**
 * @brief 解锁电机
 */
void Motor_Arm(void)
{
    motorArmed = 1;
}

/**
 * @brief 锁定电机
 */
void Motor_Disarm(void)
{
    motorArmed = 0;
    /* 注意：此处不再直接写 PWM，而是由 ISR 循环检测到 armed=0 后统一写 MIN */
    /* 满足 "Single Context Write" 约束 */
}

/**
 * @brief 检查解锁状态
 */
uint8_t Motor_IsArmed(void)
{
    return motorArmed;
}

/**
 * @brief 混控计算 (增强版 - 带解饱和)
 * @note  Mixer Desaturation: 当电机饱和时，优先保证 Roll/Pitch 控制
 * 
 *   M1(CCW)  M2(CW)
 *      ╲    ╱
 *       ╲  ╱
 *        ╲╱
 *        ╱╲
 *       ╱  ╲
 *      ╱    ╲
 *   M4(CW)  M3(CCW)
 * 
 * Roll+  : M2,M3↑ M1,M4↓ (向右倾)
 * Pitch+ : M3,M4↑ M1,M2↓ (向后仰)
 * Yaw+   : M1,M3↑ M2,M4↓ (逆时针转)
 */
void Motor_Mix(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw,
               MotorOutput_t *output)
{
    int32_t m1, m2, m3, m4;
    
    /* X型混控公式 */
    m1 = throttle - roll + pitch + yaw;  // 右前 (CCW)
    m2 = throttle + roll + pitch - yaw;  // 左前 (CW)
    m3 = throttle + roll - pitch + yaw;  // 左后 (CCW)
    m4 = throttle - roll - pitch - yaw;  // 右后 (CW)
    
    /* ==================== Mixer Desaturation ==================== */
    /* 当某电机超出范围时，整体调整以保持姿态控制优先级 */
    
    /* 1. 找到最大和最小值 */
    int32_t maxMotor = m1;
    int32_t minMotor = m1;
    if (m2 > maxMotor) maxMotor = m2;
    if (m3 > maxMotor) maxMotor = m3;
    if (m4 > maxMotor) maxMotor = m4;
    if (m2 < minMotor) minMotor = m2;
    if (m3 < minMotor) minMotor = m3;
    if (m4 < minMotor) minMotor = m4;
    
    /* 2. 计算需要的偏移量 */
    int32_t range = 1000;  // PWM 范围 (1000-2000 = 1000)
    int32_t motorRange = maxMotor - minMotor;
    
    if (motorRange > range) {
        /* 控制量太大，需要缩放 (优先保持 Roll/Pitch) */
        float scale = (float)range / (float)motorRange;
        
        /* 缩放姿态控制量 (保持油门不变) */
        m1 = throttle + (int32_t)((m1 - throttle) * scale);
        m2 = throttle + (int32_t)((m2 - throttle) * scale);
        m3 = throttle + (int32_t)((m3 - throttle) * scale);
        m4 = throttle + (int32_t)((m4 - throttle) * scale);
        
        /* 重新计算边界 */
        maxMotor = m1;
        minMotor = m1;
        if (m2 > maxMotor) maxMotor = m2;
        if (m3 > maxMotor) maxMotor = m3;
        if (m4 > maxMotor) maxMotor = m4;
        if (m2 < minMotor) minMotor = m2;
        if (m3 < minMotor) minMotor = m3;
        if (m4 < minMotor) minMotor = m4;
    }
    
    /* 3. 整体偏移，避免超出边界 */
    int32_t offset = 0;
    if (maxMotor > range) {
        offset = range - maxMotor;
    } else if (minMotor < 0) {
        offset = -minMotor;
    }
    
    m1 += offset;
    m2 += offset;
    m3 += offset;
    m4 += offset;
    
    /* 转换为PWM值 */
    m1 = MOTOR_PWM_MIN + m1;
    m2 = MOTOR_PWM_MIN + m2;
    m3 = MOTOR_PWM_MIN + m3;
    m4 = MOTOR_PWM_MIN + m4;
    
    /* 最终限幅 (安全保障) */
    if (m1 < MOTOR_PWM_MIN) m1 = MOTOR_PWM_MIN;
    if (m1 > MOTOR_PWM_MAX) m1 = MOTOR_PWM_MAX;
    if (m2 < MOTOR_PWM_MIN) m2 = MOTOR_PWM_MIN;
    if (m2 > MOTOR_PWM_MAX) m2 = MOTOR_PWM_MAX;
    if (m3 < MOTOR_PWM_MIN) m3 = MOTOR_PWM_MIN;
    if (m3 > MOTOR_PWM_MAX) m3 = MOTOR_PWM_MAX;
    if (m4 < MOTOR_PWM_MIN) m4 = MOTOR_PWM_MIN;
    if (m4 > MOTOR_PWM_MAX) m4 = MOTOR_PWM_MAX;
    
    /* ==================== 推力线性化 (Thrust Linearization) ==================== */
    /* 问题: 电机推力 ∝ 转速², 而 PWM ∝ 转速，导致推力曲线非线性
     * 解决: 对油门值进行 sqrt 变换，使 PID 看到线性的推力响应
     * 公式: linearized = sqrt(normalized) * range
     */
    #define THRUST_LINEAR_ENABLE  1
    
    #if THRUST_LINEAR_ENABLE
    {
        /* 转换为 0-1000 范围 */
        int32_t val1 = m1 - MOTOR_PWM_MIN;
        int32_t val2 = m2 - MOTOR_PWM_MIN;
        int32_t val3 = m3 - MOTOR_PWM_MIN;
        int32_t val4 = m4 - MOTOR_PWM_MIN;
        
        /* 限制范围 (Clamping) */
        if (val1 < 0) val1 = 0; else if (val1 > 1000) val1 = 1000;
        if (val2 < 0) val2 = 0; else if (val2 > 1000) val2 = 1000;
        if (val3 < 0) val3 = 0; else if (val3 > 1000) val3 = 1000;
        if (val4 < 0) val4 = 0; else if (val4 > 1000) val4 = 1000;
        
        /* [Phase-3] 查表替代 sqrtf (14 cycles -> 2 cycles) */
        m1 = MOTOR_PWM_MIN + thrustLinearTable[val1];
        m2 = MOTOR_PWM_MIN + thrustLinearTable[val2];
        m3 = MOTOR_PWM_MIN + thrustLinearTable[val3];
        m4 = MOTOR_PWM_MIN + thrustLinearTable[val4];
    }
    #endif
    
    /* ==================== Airmode ==================== */
    /* 解锁后即使油门为0也保持姿态控制余量 */
    #define AIRMODE_ENABLE      1
    #define MOTOR_IDLE_OFFSET   60  // 最小提升量
    
    #if AIRMODE_ENABLE
    if (motorArmed && throttle < 50) {
        /* 找到最小电机值 */
        int32_t minMotor = m1;
        if (m2 < minMotor) minMotor = m2;
        if (m3 < minMotor) minMotor = m3;
        if (m4 < minMotor) minMotor = m4;
        
        /* 如果最小值低于怠速, 整体提升 */
        int32_t minIdle = MOTOR_PWM_MIN + MOTOR_IDLE_OFFSET;
        if (minMotor < minIdle) {
            int32_t boost = minIdle - minMotor;
            m1 += boost;
            m2 += boost;
            m3 += boost;
            m4 += boost;
            
            /* 再次限幅 */
            if (m1 > MOTOR_PWM_MAX) m1 = MOTOR_PWM_MAX;
            if (m2 > MOTOR_PWM_MAX) m2 = MOTOR_PWM_MAX;
            if (m3 > MOTOR_PWM_MAX) m3 = MOTOR_PWM_MAX;
            if (m4 > MOTOR_PWM_MAX) m4 = MOTOR_PWM_MAX;
        }
    }
    #endif
    
    output->motor[MOTOR_1] = (uint16_t)m1;
    output->motor[MOTOR_2] = (uint16_t)m2;
    output->motor[MOTOR_3] = (uint16_t)m3;
    output->motor[MOTOR_4] = (uint16_t)m4;
    output->armed = motorArmed;
}

/* ==================== 定时器配置 ==================== */

/**
 * @brief TIM1 PWM初始化
 * @note  170MHz系统时钟 -> 400Hz PWM
 *        ARR = 2500-1 (周期2.5ms = 400Hz)
 *        PSC = 170-1 (1us分辨率)
 */
static void TIM1_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能时钟 */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置GPIO为复用功能 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置TIM1基本参数 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 170 - 1;         // 170MHz / 170 = 1MHz (1us)
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 2500 - 1;           // 1MHz / 2500 = 400Hz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        while(1);
    }
    
    /* 配置PWM通道 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = MOTOR_PWM_MIN;        // 初始值1000us
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    
    /* [Phase-2] 启用 OC Preload：CCR 更新在下个周期生效，避免 Glitch */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    /* TIM1是高级定时器，需要使能MOE */
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    /* [PWM Sync] 启用TIM1更新中断 */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2, 0);  // 优先级2 (低于2kHz IMU中断)
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

/**
 * @brief TIM4 PWM初始化 (用于M4 - PB6)
 * @note  PA11 有 USB 冲突，所以 M4 使用 PB6
 */
static void TIM4_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置GPIO PB6 -> TIM4_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 配置TIM4基本参数 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 170 - 1;         // 170MHz / 170 = 1MHz
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 2500 - 1;           // 400Hz
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        while(1);
    }
    
    /* 配置PWM通道 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = MOTOR_PWM_MIN;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    
    /* [Phase-2] 启用 OC Preload */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}


/* ==================== [PWM Sync] API实现 ==================== */

void Motor_SetPendingPWM(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    g_pendingPWM[0] = m1;
    g_pendingPWM[1] = m2;
    g_pendingPWM[2] = m3;
    g_pendingPWM[3] = m4;
    g_pwmReady = 1;  // 原子标志
}

void Motor_GetSyncStats(uint32_t *updateCount) {
    *updateCount = g_pwmUpdateCount;
}

/* ==================== [PWM Sync] TIM中断处理 ==================== */

void TIM1_UP_TIM16_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        
        g_pwmDivider++;
        if (g_pwmDivider >= 5) {  // 2kHz / 5 = 400Hz
            g_pwmDivider = 0;
            
            if (g_pwmReady) {
                // 原子更新PWM寄存器
                TIM1->CCR1 = g_pendingPWM[0];
                TIM1->CCR2 = g_pendingPWM[1];
                TIM1->CCR3 = g_pendingPWM[2];
                TIM4->CCR1 = g_pendingPWM[3];
                
                g_pwmReady = 0;
                g_pwmUpdateCount++;
            }
        }
    }
}
