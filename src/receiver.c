/**
 * @file receiver.c
 * @brief PPM接收机驱动实现
 * @note  使用TIM2输入捕获解析PPM信号
 * 
 * 引脚: PA0 (TIM2_CH1)
 * 
 * PPM信号格式:
 * __|¯¯¯|__|¯¯¯¯¯|__|¯¯¯|__|¯¯¯¯¯¯¯¯¯¯|__
 *    CH1     CH2    CH3      SYNC
 * 
 * - 每个通道是一个高电平脉冲 (1000~2000us)
 * - 同步脉冲 >3000us 表示一帧结束
 */

#include "receiver.h"
#include "stm32g4xx_hal.h"

/* ==================== 私有变量 ==================== */

static TIM_HandleTypeDef htim2;
static volatile uint16_t ppmBuffer[RC_CH_COUNT];    // PPM捕获缓冲
static volatile uint8_t  ppmIndex = 0;              // 当前通道索引
static volatile uint32_t lastCaptureTime = 0;       // 上次捕获时间
static volatile uint32_t lastFrameTime = 0;         // 上次完整帧时间
static volatile bool     frameValid = false;        // 帧有效标志

static Receiver_t rcData = {0};

/* ==================== 私有函数 ==================== */

static void TIM2_IC_Init(void);

/* ==================== API实现 ==================== */

/**
 * @brief 初始化PPM接收
 */
void Receiver_Init(void)
{
    /* 初始化通道值为中位 */
    for (int i = 0; i < RC_CH_COUNT; i++) {
        rcData.channel[i] = RC_PWM_MID;
        ppmBuffer[i] = RC_PWM_MID;
    }
    /* 油门初始为最低 */
    rcData.channel[RC_CH_THROTTLE] = RC_PWM_MIN;
    ppmBuffer[RC_CH_THROTTLE] = RC_PWM_MIN;
    
    rcData.connected = false;
    rcData.lastUpdateTime = 0;
    rcData.frameCount = 0;
    
    TIM2_IC_Init();
}

/**
 * @brief 更新接收机状态
 */
void Receiver_Update(void)
{
    uint32_t now = HAL_GetTick();
    
    /* 检查信号超时 */
    if (now - rcData.lastUpdateTime > RC_TIMEOUT_MS) {
        rcData.connected = false;
    }
    
    /* 如果有新的有效帧 */
    if (frameValid) {
        frameValid = false;
        
        /* 复制数据 */
        for (int i = 0; i < RC_CH_COUNT; i++) {
            /* 限幅 */
            uint16_t val = ppmBuffer[i];
            if (val < RC_PWM_MIN) val = RC_PWM_MIN;
            if (val > RC_PWM_MAX) val = RC_PWM_MAX;
            rcData.channel[i] = val;
        }
        
        rcData.lastUpdateTime = now;
        rcData.connected = true;
        rcData.frameCount++;
    }
}

/**
 * @brief 获取通道值
 */
uint16_t Receiver_GetChannel(uint8_t ch)
{
    if (ch >= RC_CH_COUNT) return RC_PWM_MID;
    
    if (!rcData.connected) {
        /* 失联时返回安全值 */
        if (ch == RC_CH_THROTTLE) return RC_PWM_MIN;
        return RC_PWM_MID;
    }
    
    return rcData.channel[ch];
}

/**
 * @brief 获取连接状态
 */
bool Receiver_IsConnected(void)
{
    return rcData.connected;
}

/**
 * @brief 获取油门值 (0~1000)
 */

/* RC 输入优化参数 */
#define RC_EXPO         0.3f    // Expo 曲线系数 (0=线性, 1=最大曲线)
#define RC_DEADZONE     20      // 死区 (摇杆中位附近不响应)
#define RC_FILTER_COEF  0.7f    // 低通滤波系数 (越大越平滑，越小越灵敏)

/* 滤波器状态 */
static float rollFiltered = 0.0f;
static float pitchFiltered = 0.0f;
static float yawFiltered = 0.0f;

/**
 * @brief 应用 Expo 曲线
 * @note  让中位附近更平缓，边缘更灵敏
 */
static float ApplyExpo(float input, float expo)
{
    // Expo 曲线公式: output = input * (1 - expo + expo * input^2)
    float absInput = (input < 0) ? -input : input;
    float output = input * (1.0f - expo + expo * absInput * absInput / 250000.0f);
    return output;
}

/**
 * @brief 应用死区
 */
static int16_t ApplyDeadzone(int16_t input, int16_t deadzone)
{
    if (input > deadzone) {
        return input - deadzone;
    } else if (input < -deadzone) {
        return input + deadzone;
    } else {
        return 0;
    }
}

int16_t Receiver_GetThrottle(void)
{
    uint16_t ch = Receiver_GetChannel(RC_CH_THROTTLE);
    int16_t val = (int16_t)ch - RC_PWM_MIN;
    if (val < 0) val = 0;
    if (val > 1000) val = 1000;
    return val;
}

/**
 * @brief 获取横滚控制量 (-500~500) 带 Expo + 死区 + 滤波
 */
int16_t Receiver_GetRoll(void)
{
    uint16_t ch = Receiver_GetChannel(RC_CH_ROLL);
    int16_t val = (int16_t)ch - RC_PWM_MID;
    if (val > 500) val = 500;
    if (val < -500) val = -500;
    
    // 应用死区
    val = ApplyDeadzone(val, RC_DEADZONE);
    
    // 应用 Expo 曲线
    float valFloat = ApplyExpo((float)val, RC_EXPO);
    
    // 低通滤波
    rollFiltered = rollFiltered * RC_FILTER_COEF + valFloat * (1.0f - RC_FILTER_COEF);
    
    return (int16_t)rollFiltered;
}

/**
 * @brief 获取俯仰控制量 (-500~500) 带 Expo + 死区 + 滤波
 */
int16_t Receiver_GetPitch(void)
{
    uint16_t ch = Receiver_GetChannel(RC_CH_PITCH);
    int16_t val = (int16_t)ch - RC_PWM_MID;
    if (val > 500) val = 500;
    if (val < -500) val = -500;
    
    // 应用死区
    val = ApplyDeadzone(val, RC_DEADZONE);
    
    // 应用 Expo 曲线
    float valFloat = ApplyExpo((float)val, RC_EXPO);
    
    // 低通滤波
    pitchFiltered = pitchFiltered * RC_FILTER_COEF + valFloat * (1.0f - RC_FILTER_COEF);
    
    return (int16_t)pitchFiltered;
}

/**
 * @brief 获取偏航控制量 (-500~500) 带 Expo + 死区 + 滤波
 */
int16_t Receiver_GetYaw(void)
{
    uint16_t ch = Receiver_GetChannel(RC_CH_YAW);
    int16_t val = (int16_t)ch - RC_PWM_MID;
    if (val > 500) val = 500;
    if (val < -500) val = -500;
    
    // 应用死区
    val = ApplyDeadzone(val, RC_DEADZONE);
    
    // 应用 Expo 曲线
    float valFloat = ApplyExpo((float)val, RC_EXPO);
    
    // 低通滤波
    yawFiltered = yawFiltered * RC_FILTER_COEF + valFloat * (1.0f - RC_FILTER_COEF);
    
    return (int16_t)yawFiltered;
}

/**
 * @brief 检查解锁请求 (油门最低 + 偏航最右)
 */
bool Receiver_IsArmRequest(void)
{
    if (!rcData.connected) return false;
    
    uint16_t thr = rcData.channel[RC_CH_THROTTLE];
    uint16_t yaw = rcData.channel[RC_CH_YAW];
    
    /* 油门 < 1050 且 偏航 > 1900 */
    return (thr < 1050 && yaw > 1900);
}

/**
 * @brief 检查锁定请求 (油门最低 + 偏航最左)
 */
bool Receiver_IsDisarmRequest(void)
{
    if (!rcData.connected) return false;
    
    uint16_t thr = rcData.channel[RC_CH_THROTTLE];
    uint16_t yaw = rcData.channel[RC_CH_YAW];
    
    /* 油门 < 1050 且 偏航 < 1100 */
    return (thr < 1050 && yaw < 1100);
}

/* ==================== 定时器配置 ==================== */

/**
 * @brief TIM2输入捕获初始化
 * @note  PA0 -> TIM2_CH1
 *        时钟: 170MHz, 预分频170 -> 1MHz (1us分辨率)
 */
static void TIM2_IC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    /* 使能时钟 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* 配置PA0为复用功能 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置TIM2基本参数 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 170 - 1;         // 170MHz / 170 = 1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;         // 32位计数器最大值
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
        while(1);
    }
    
    /* 配置输入捕获通道1 */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;  // 上升沿捕获
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x0F;  // 滤波
    
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        while(1);
    }
    
    /* 使能捕获中断 */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    /* 启动输入捕获 */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief TIM2中断处理 (PPM解析)
 */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
        
        uint32_t capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
        uint32_t pulseWidth = capture - lastCaptureTime;
        lastCaptureTime = capture;
        
        /* 判断脉宽 */
        if (pulseWidth > 3000) {
            /* 同步脉冲，一帧结束 */
            if (ppmIndex >= RC_CH_COUNT) {
                frameValid = true;
                lastFrameTime = HAL_GetTick();
            }
            ppmIndex = 0;
        }
        else if (pulseWidth >= 800 && pulseWidth <= 2200) {
            /* 有效通道脉冲 */
            if (ppmIndex < RC_CH_COUNT) {
                ppmBuffer[ppmIndex] = (uint16_t)pulseWidth;
                ppmIndex++;
            }
        }
        /* 其他脉宽忽略 */
    }
}
