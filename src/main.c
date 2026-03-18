/**
 * @file main.c
 * @brief STM32G431飞控主程序 - 完整版
 * @date 2026-01-05
 * 
 * 功能：
 * - 读取ICM42688六轴数据
 * - Madgwick姿态解算 
 * - PPM遥控器接收
 * - PID控制 + 电机混控
 */

#include "stm32g4xx_hal.h"
#include "system_config.h"
#include "bsp_uart.h"
#include "icm42688.h"
#include "bsp_spi.h"
#include "attitude.h"
#include "pid.h"
#include "motor.h"
#include "receiver.h"
#include "cli.h"
#include "config.h"
#include "battery.h"
#include "buzzer.h"
#include "dshot.h"
#include "drdy_imu.h"
#include "ms5611.h"
#include "alt_hold.h"
#include "stm32g4xx_hal_iwdg.h"
#include <stdio.h>
#include <math.h>

/* ==================== 宏定义 ==================== */
#define DEG_TO_RAD  0.017453292f
#define RAD_TO_DEG  57.29577951f

/* 最大倾斜角度限制 (度) */
#define MAX_ANGLE   45.0f

/* ==================== 全局变量 ==================== */
// static ICM42688_Data_t imuData; // [Phase-1.5] Removed
/* 看门狗句柄 */
static IWDG_HandleTypeDef hiwdg;
static Attitude_t attitude;
static FlightPID_t flightPID;
static MotorOutput_t motorOutput;

static uint32_t loopCounter = 0;
static uint32_t lastPrintTime = 0;
static uint32_t lastBuzzerTime = 0;  // 蜂鸣器计时

uint8_t armed = 0;           // 解锁状态 (Global for CLI access)
static uint32_t armRequestTime = 0; // 解锁请求开始时间

/* 全局调试标志 */
volatile bool accAttActive = false;

/* [Optimal] Angle 分频计数 (用于串口验证) */
uint32_t angleLoopCount = 0;

/* ==================== 函数声明 ==================== */
static void LED_Init(void);
static void LED_Toggle(void);
static void MX_IWDG_Init(void);
static void FlightControl(DRDY_IMU_Data_t *imu, float dt);

/**
 * @brief 主函数
 */
int main(void)
{
    /* HAL初始化 */
    HAL_Init();
    
    /* 优先级分组: 4 bits for pre-emption (0-15), 0 bits for subpriority */
    /* 这是 STM32 HAL 默认，但显式调用更安全 */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    
    /* SysTick 设为最低优先级 (15)，防止打断 2kHz 控制回路 */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
    
    /* 配置系统时钟到170MHz */
    SystemClock_Config();
    
    /* 初始化LED */
    LED_Init();
    
    /* 初始化调试串口 */
    BSP_UART2_Init(DEBUG_BAUDRATE);
    
    BSP_UART_Printf("\r\n");
    BSP_UART_Printf("========================================\r\n");
    BSP_UART_Printf("  STM32G431 Flight Controller v1.0\r\n");
    BSP_UART_Printf("  By Gemini AI Assistant\r\n");
    BSP_UART_Printf("========================================\r\n\r\n");
    
    /* 初始化硬件SPI */
    BSP_SPI1_Init();
    BSP_UART_Printf("[SPI] Hardware SPI Init OK\r\n");
    
    /* 检查复位源 (是否由看门狗复位?) */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        BSP_UART_Printf("\r\n[SYSTEM] !!! WARNING: Recovered from IWDG / WATCHDOG RESET !!!\r\n");
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
    
    /* 初始化ICM42688 */
    // Moved to later

    /* [Phase-3] 看门狗已移至所有传感器初始化之后，防止启动超时 */
    // MX_IWDG_Init(); // Moved down
    
    /* ============================================================ */
    /*               正式飞行程序初始化流程                       */
    /* ============================================================ */
    
    /* 1. IMU 检测与配置 (使用 SPI1) */
    
    // --- 接线调试辅助循环 ---
    // 持续尝试 50 次 (约5秒)，方便用户检查接线
    BSP_UART_Printf("[IMU] Waiting for device... (Check wiring: CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7)\r\n");
    uint8_t whoami = 0;
    for (int i = 0; i < 50; i++) {
        ICM42688_CS_LOW();
        uint8_t tx_check = 0x80 | 0x75; 
        BSP_SPI1_Transmit(&tx_check, 1);
        BSP_SPI1_Receive(&whoami, 1);
        ICM42688_CS_HIGH();
        
        if (whoami == 0x47) {
            BSP_UART_Printf("[IMU] FOUND! (0x47) at attempt %d\r\n", i+1);
            break;
        } else {
            if (i % 5 == 0) // 每5次打印一次，避免刷屏太快
                BSP_UART_Printf("[IMU] Not found. Read: 0x%02X\r\n", whoami);
            HAL_Delay(100);
        }
    }
    // --- 结束调试循环 ---
    
    if (whoami != 0x47) {
        BSP_UART_Printf("[IMU] WARNING! Giving up. (Last read: 0x%02X)\r\n", whoami);
        // 即使没找到也继续，以免阻塞气压计调试
    } else {
        BSP_UART_Printf("[IMU] OK (0x%02X)\r\n", whoami);
        
        // 调用驱动库进行完整配置
        ICM42688_Configure(IMU_GYRO_FS, IMU_ACCEL_FS, IMU_ODR);
        
        BSP_UART_Printf("[IMU] Calibrating... Keep flat and still!\r\n");
        ICM42688_Calibrate();
        BSP_UART_Printf("[IMU] Calibration DONE.\r\n");
    }
    
    /* 2. 初始化姿态解算 */
    Attitude_Init(2000.0f);  // 2kHz姿态融合 (最优延迟配置)
    Attitude_SetBeta(0.5f);  // 增大 Beta 加速漂移修正
    BSP_UART_Printf("[ATT] Madgwick OK\r\n");
    
    /* 3. 初始化PID */
    FlightPID_Init(&flightPID);
    
    /* 3.5 初始化 FFT Notch */
    #include "fft_notch.h"
    #if USE_FFT_NOTCH
    FFTNotch_Init();
    BSP_UART_Printf("[FFT] Notch Init OK\r\n");
    #endif
    
    /* 尝试从Flash加载参数 */
    if (Config_Load(&flightPID)) {
        BSP_UART_Printf("[PID] Loaded from Flash\r\n");
    } else {
        BSP_UART_Printf("[PID] Using default params\r\n");
    }
    
    /* 4. 初始化电机 */
    if (USE_DSHOT) {
        DShot_Init();
        BSP_UART_Printf("[MTR] DShot600 Init OK (PA8-10, PB6)\r\n");
    } else {
        Motor_Init();
        BSP_UART_Printf("[MTR] PWM 400Hz Init OK (PA8-11)\r\n");
    }

    /* 5. 初始化电池检测 */
    Battery_Init();
    BSP_UART_Printf("[BAT] ADC Init (PA1)\r\n");
    
    /* 6. 初始化接收机 */
    Receiver_Init();
    BSP_UART_Printf("[RC]  PPM OK (PA0)\r\n");
    
    /* 7. 初始化CLI调参 */
    CLI_Init(&flightPID);
    BSP_UART_Printf("[CLI] OK (type HELP)\r\n");
    
    BSP_UART_Printf("\r\n[READY] System running!\r\n");
    BSP_UART_Printf("Type HELP for CLI commands\r\n\r\n");
    
    /* 8. 初始化蜂鸣器 */
    Buzzer_Init();
    BSP_UART_Printf("[BUZ] Buzzer Init (PC14)\r\n");
    
    BSP_UART_Printf("[INFO] Stuttering fix active. Armed idle speed set.\r\n");
    
    /* 9. 初始化 MS5611 气压计 (硬件 I2C1: PA15=SCL, PB7=SDA) */
    /* 注意：必须在 DRDY 中断启动前初始化，否则 I2C 会被 2kHz 中断打断导致初始化失败 */
    BSP_UART_Printf("[BARO] Trying MS5611 on I2C1 (PA15/PB7)...\r\n");
    if (MS5611_Init()) {
        BSP_UART_Printf("[BARO] MS5611 OK!\r\n");
        AltHold_Init();
    } else {
        BSP_UART_Printf("[BARO] MS5611 NOT FOUND! Alt-hold disabled.\r\n");
    }

    /* 10. 初始化 DRDY 中断驱动 (2kHz) */
    DRDY_IMU_Init();
    BSP_UART_Printf("[DRDY] 2kHz Interrupt-Driven IMU Init OK\r\n");
    
    /* 启动 DRDY 中断 (开始高频循环) */
    DRDY_IMU_Start();
    BSP_UART_Printf("[DRDY] EXTI1 (PB1) Started - 2kHz Control Active\r\n");
    
    /* 启动音: 滴-滴 */
    Buzzer_Beep(2, 100);
    
    /* [Phase-3] 初始化看门狗 (100ms 超时) - 移至最后，防止传感器初始化阻塞触发复位 */
    BSP_UART_Printf("[IWDG] Watchdog Enabled (100ms)\r\n");
    MX_IWDG_Init();
    
    /* ==================== 主循环 ==================== */
    uint32_t lastLoopTime = HAL_GetTick();
    
    while (1)
    {
        uint32_t now = HAL_GetTick();
        
        /* 1ms周期 (1000Hz) */
        if (now - lastLoopTime >= 1)
        {
            lastLoopTime = now;
            loopCounter++;
            
            /* 1. 更新遥控器 */
            Receiver_Update();
            
            /* 2. IMU 数据由 2kHz DRDY ISR 更新，Main Loop 仅读取缓存用于显示 */
            /* [Phase-1.5] 主循环禁止主动读取 IMU */
            DRDY_IMU_Data_t *drdyImu = DRDY_IMU_GetData();
            
            /* 3. 姿态解算 (已移至 2kHz ISR) */
            // Attitude_Update(gx, gy, gz, imuData.accel.x, imuData.accel.y, imuData.accel.z);
            Attitude_GetEuler(&attitude); // 仅读取用于显示
            
            /* 计算垂直加速度 (用于定高显示，使用 DRDY 缓存数据) */
            // float accZ_G = Attitude_GetEarthG(drdyImu->accel[0], drdyImu->accel[1], drdyImu->accel[2]);
            /* [Phase-3] AltHold 逻辑现在由 主循环 Correct + ISR Predict 组成 */

            /* 4. 解锁/锁定逻辑 */
            if (!armed) {
                if (Receiver_IsArmRequest()) {
                    if (armRequestTime == 0) {
                        armRequestTime = now;
                    } else if (now - armRequestTime > 2000) {
                        /* 保持2秒解锁 */
                        armed = 1;
                        Motor_Arm();
                        BSP_UART_Printf("[ARM] ARMED! (2kHz Control)\r\n");
                        Buzzer_Beep(3, 80);
                        armRequestTime = 0;
                    }
                } else {
                    armRequestTime = 0;
                }
            } else {
                if (Receiver_IsDisarmRequest()) {
                    armed = 0;
                    Motor_Disarm();
                    BSP_UART_Printf("[ARM] DISARMED\r\n");
                    Buzzer_Beep(1, 300);
                }
            }
            
            /* 5. Failsafe Logic (仅负责状态切换) */
            if (armed && !Receiver_IsConnected()) {
                /* 遥控器断开 -> 强制锁定 */
                armed = 0;
                Motor_Disarm(); // 仅设置标志位，不写 PWM
                BSP_UART_Printf("[FAILSAFE] RC Lost! Disarmed.\r\n");
                Buzzer_Beep(5, 50);
            }
            
            /* 注意：电机输出全权移交 2kHz ISR */
            /* Main Loop 禁止调用 Motor_SetAllPWM */
            
            /* 5. 检查接收机连接与模式 (50Hz) */
            static uint32_t lastRcCheck = 0;
            if (now - lastRcCheck >= 20) {
                lastRcCheck = now;
                Receiver_Update();
                
                /* [Mode] 定高开关 logic (AUX1 > 1500) */
                int16_t aux1 = Receiver_GetChannel(RC_CH_AUX1);
                static bool altHoldActive = false;
                
                if (aux1 > 1600 && !altHoldActive && armed) {
                    /* 激活定高: 锁住当前高度 */
                    AltHold_Enable(true);
                    MS5611_Data_t *baro = MS5611_GetData();
                    if (baro->valid) {
                        AltHold_SetTarget(baro->altitude);
                        BSP_UART_Printf("[MODE] AltHold ON. Target=%.2fm\r\n", baro->altitude);
                    }
                    altHoldActive = true;
                }
                else if (aux1 < 1400 && altHoldActive) {
                    /* 关闭定高 */
                    AltHold_Enable(false);
                    BSP_UART_Printf("[MODE] AltHold OFF.\r\n");
                    altHoldActive = false;
                }
                
                /* 解锁逻辑 */
                if (Receiver_IsArmRequest() && !armed) {
                    if (armRequestTime == 0) {
                        armRequestTime = now;
                    } else if (now - armRequestTime > 2000) {
                        armed = 1;
                        Motor_Arm();
                        BSP_UART_Printf("[ARM] ARMED! (2kHz Control)\r\n");
                        Buzzer_Beep(3, 80);
                        armRequestTime = 0;
                    }
                } else {
                    armRequestTime = 0;
                }
            }
            
            /* 6. 更新气压计 (50Hz) */
            static uint32_t lastBaroTime = 0;
            if (now - lastBaroTime >= 20) {
                lastBaroTime = now;
                MS5611_Update();
                
                /* 更新定高 */
                MS5611_Data_t *baro = MS5611_GetData();
                if (baro->valid) {
                    AltHold_Correct(baro->altitude, 0.02f);
                }
                
                /* [动态 Notch] FFT 自适应双 Notch (50Hz 调度) */
                #include "fft_notch.h"
                #if USE_FFT_NOTCH
                FFTNotch_Update();
                #endif
            }
            
            /* [Phase-3] 喂狗 (Watchdog Refresh) */
            HAL_IWDG_Refresh(&hiwdg);
            
            /* [Optimal-A] 计算 p99 (5Hz) */
            static uint32_t lastP99Time = 0;
            if (now - lastP99Time >= 200) {
                lastP99Time = now;
                DRDY_IMU_CalcP99();
            }
            
            /* 7. 串口输出 (5Hz) - [增强版] */
            if (now - lastPrintTime >= 200)
            {
                lastPrintTime = now;
                
                /* 获取统计 */
                DRDY_Stats_t *stats = DRDY_IMU_GetStats();
                
                /* 基础信息 */
                BSP_UART_Printf("R:%d P:%d Y:%d T:%d %s\r\n",
                    (int)attitude.roll, (int)attitude.pitch, (int)attitude.yaw,
                    Receiver_GetThrottle(),
                    armed ? "[ARMED]" : "");
                
                /* [关键] ISR 性能监控 */
                BSP_UART_Printf("  ISR: avg=%uus max=%uus p99=%uus overbudget=%u\r\n",
                    stats->avgExecTime_us, stats->maxExecTime_us,
                    stats->p99ExecTime_us, stats->isrOverbudget);
                
                /* [关键] dt 统计 */
                BSP_UART_Printf("  dt: min=%uus max=%uus bad_dt=%u\r\n",
                    stats->minLoopTime_us, stats->maxLoopTime_us,
                    stats->bad_dt_count);
                
                /* [验证] Main Loop 频率 (理论1000Hz,实际受串口/I2C影响) */
                BSP_UART_Printf("  Main Loop: %u Hz (target 1000Hz)\r\n", loopCounter * 5);
                loopCounter = 0;
                
                /* [Debug] FFT Dual Notch Status */
                #if USE_FFT_NOTCH
                extern float fft_peak_freq1, fft_peak_freq2;
                extern float fft_peak_power1, fft_peak_power2;
                extern uint32_t fft_update_count;
                BSP_UART_Printf("  Notch: f1=%.0f f2=%.0f p1=%.0f p2=%.0f upd=%u\r\n", 
                    fft_peak_freq1, fft_peak_freq2, fft_peak_power1, fft_peak_power2, fft_update_count);
                #else
                extern float debug_peak_freq;
                extern float debug_peak_energy;
                extern uint32_t notch_update_count;
                BSP_UART_Printf("  Notch: peak=%.1fHz eng=%.0f upd=%u\r\n", 
                    debug_peak_freq, debug_peak_energy, notch_update_count);
                #endif
            }
            
            /* 蜂鸣器报警逻辑 */
            if (Battery_IsLowVoltage()) {
                Buzzer_Toggle();
            } else if (!Receiver_IsConnected() && armed) {
                Buzzer_On();
            } else {
                Buzzer_Off();
            }
            
            LED_Toggle();
        }
    }
}

/**
 * @brief 飞行控制 (串级PID)
 * @note  循环频率 1000Hz (1ms) - 适配 generic 30A ESC
 */
static void FlightControl(DRDY_IMU_Data_t *imu, float dt)
{
    // float dt = 0.001f; // 移除，由参数传入
    static uint16_t failsafeCounter = 0;
    static int16_t failsafeThrottle = 0;
    
    /* 获取遥控器输入 */
    int16_t rcThrottle = Receiver_GetThrottle();
    int16_t rcRoll     = Receiver_GetRoll();
    int16_t rcPitch    = Receiver_GetPitch();
    int16_t rcYaw      = Receiver_GetYaw();
    
    /* ==================== 动态参数调整 ==================== */
    /* 提升响应速度的 Beta 值 (如果震动大可降回 0.05) */
    Attitude_SetBeta(0.1f);
    
    /* ==================== [Phase-2.1] Dynamic Idle ==================== */
    /* 低油门时检测陀螺噪声，自动提升电机最低转速防失速 */
    static float motorIdleOffset = 0;
    
    if (rcThrottle < 100) {  // 低油门区间
        float gyroNoise = fabsf(imu->gyro[0]) + fabsf(imu->gyro[1]) + fabsf(imu->gyro[2]);
        if (gyroNoise > 50.0f) {  // deg/s
            motorIdleOffset = 50;  // 提升怠速PWM
        } else {
            motorIdleOffset *= 0.98f;  // 慢慢降回
        }
    } else {
        motorIdleOffset = 0;  // 正常油门时禁用
    }
    
    /* ==================== [Phase-2.2] Dynamic Gyro LPF ==================== */
    /* 根据油门动态调整陀螺仪LPF截止频率 */
    float throttlePercent = (float)rcThrottle / 10.0f;  // 0-1000 → 0-100
    DRDY_IMU_SetDynamicGyroLPF(throttlePercent);
    
    /* ==================== 失控保护 ==================== */
    if (!Receiver_IsConnected()) {
        failsafeCounter++;
        if (failsafeCounter > 500) {  // 0.5秒 @ 1000Hz
            if (failsafeCounter == 501) failsafeThrottle = rcThrottle;
            if (failsafeCounter % 100 == 0) { // 每100ms
                failsafeThrottle -= 20;
                if (failsafeThrottle < 0) failsafeThrottle = 0;
            }
            rcThrottle = failsafeThrottle;
            rcRoll = 0; rcPitch = 0; rcYaw = 0;
        }
    } else {
        failsafeCounter = 0;
    }
    
    /* ==================== 低压保护 ==================== */
    if (Battery_IsLowVoltage()) {
        if (rcThrottle > 600) rcThrottle = 600;
    }
    
    /* ==================== TPA (油门PID衰减) ==================== */
    float tpaFactor = 1.0f;
    if (rcThrottle > 500) {
        tpaFactor = 1.0f - 0.3f * (float)(rcThrottle - 500) / 500.0f;
    }
    
    /* 临时调整 PID 参数 */
    float orig_Kp_roll = flightPID.roll_rate.Kp;
    float orig_Kd_roll = flightPID.roll_rate.Kd;
    float orig_Kp_pitch = flightPID.pitch_rate.Kp;
    float orig_Kd_pitch = flightPID.pitch_rate.Kd;
    
    flightPID.roll_rate.Kp  *= tpaFactor;
    flightPID.roll_rate.Kd  *= tpaFactor;
    flightPID.pitch_rate.Kp *= tpaFactor;
    flightPID.pitch_rate.Kd *= tpaFactor;
    
    /* ==================== [Advanced-1] D-term LPF 自适应 ==================== */
    /* 高油门时降低 D-LPF 截止频率，减少电机噪声放大 */
    /* 低油门: alpha=0.25 (cut=80Hz) | 高油门: alpha=0.1 (cut=30Hz) */
    {
        float thr_norm = (float)rcThrottle / 1000.0f;
        if (thr_norm < 0.0f) thr_norm = 0.0f;
        if (thr_norm > 1.0f) thr_norm = 1.0f;
        
        float dLpfAlpha = 0.25f - thr_norm * 0.15f;  // 0.25 -> 0.10
        flightPID.roll_rate.dLpfAlpha = dLpfAlpha;
        flightPID.pitch_rate.dLpfAlpha = dLpfAlpha;
        flightPID.yaw_rate.dLpfAlpha = dLpfAlpha;
    }
    
    /* ==================== [Advanced-2] Anti-Gravity ==================== */
    /* 油门快速变化时临时增大 I 增益，改善垂直机动姿态保持 */
    static int16_t lastThrottle = 0;
    float throttleDelta = fabsf((float)(rcThrottle - lastThrottle));
    lastThrottle = rcThrottle;
    
    float antiGravityGain = 1.0f;
    #define ANTIGRAVITY_THRESHOLD 30.0f   // 油门变化阈值 (每帧)
    #define ANTIGRAVITY_BOOST     2.5f    // 最大增益倍数
    
    if (throttleDelta > ANTIGRAVITY_THRESHOLD) {
        /* 线性插值: 30->1x, 100->2.5x */
        antiGravityGain = 1.0f + (throttleDelta - ANTIGRAVITY_THRESHOLD) / 70.0f * (ANTIGRAVITY_BOOST - 1.0f);
        if (antiGravityGain > ANTIGRAVITY_BOOST) antiGravityGain = ANTIGRAVITY_BOOST;
    }
    
    /* 临时提升 Ki (将在 PID 计算后恢复) */
    float orig_Ki_roll = flightPID.roll_rate.Ki;
    float orig_Ki_pitch = flightPID.pitch_rate.Ki;
    flightPID.roll_rate.Ki *= antiGravityGain;
    flightPID.pitch_rate.Ki *= antiGravityGain;
    
    /* ==================== [Advanced-3] RPM 滤波估算 ==================== */
    /* 基于油门估算电机转速，更精准设置 Notch 频率 */
    /* 典型曲线: idle=80Hz, full=350Hz (需校准) */
    #if !USE_FFT_NOTCH  // 仅在禁用 FFT 时使用 RPM 估算
    {
        float thr_norm = (float)rcThrottle / 1000.0f;
        if (thr_norm < 0.1f) thr_norm = 0.1f;
        
        /* 非线性映射: rpm ≈ sqrt(throttle) * k */
        float rpm_factor = sqrtf(thr_norm);
        float notch_freq = 80.0f + rpm_factor * 270.0f;  // 80-350Hz
        
        /* 平滑更新 (减少跳变) */
        static float last_notch = 150.0f;
        notch_freq = last_notch * 0.9f + notch_freq * 0.1f;
        last_notch = notch_freq;
        
        DRDY_IMU_SetNotchFreq(notch_freq, 2.5f);
    }
    #endif
    
    /* ==================== [C] AltHold 预测与控制 ==================== */
    /* 计算 Earth-Frame Z轴加速度 (扣除重力) */
    /* accZ_Earth = -accX*sinP + accY*sinR*cosP + accZ*cosR*cosP - 1G */
    float radRoll = attitude.roll * 0.01745329f;
    float radPitch = attitude.pitch * 0.01745329f;
    float sinP = sinf(radPitch);
    float cosP = cosf(radPitch);
    float sinR = sinf(radRoll);
    float cosR = cosf(radRoll);
    
    float accZ_Earth = -imu->accel[0] * sinP 
                     + imu->accel[1] * sinR * cosP 
                     + imu->accel[2] * cosR * cosP;
    accZ_Earth -= 1.0f; // 扣除重力 (1G)
    
    /* 调用预测步骤 (1kHz/2kHz) */
    AltHold_Predict(accZ_Earth, dt);
    
    /* ==================== 动态前馈 (Stick Boost) ==================== */
    /* 非线性前馈：摇杆动得越快，前馈越猛 */
    static int16_t lastRcRoll = 0, lastRcPitch = 0, lastRcYaw = 0;
    
    float deltaRoll = (float)(rcRoll - lastRcRoll);
    float deltaPitch = (float)(rcPitch - lastRcPitch);
    float deltaYaw = (float)(rcYaw - lastRcYaw);
    
    /* 基础增益 + 动态增益 (基于变化速率的平方) */
    float boostFactor = 1.0f + (fabsf(deltaRoll) * 0.05f); // 简单非线性
    float rollFF  = deltaRoll  * 0.5f * boostFactor;
    float pitchFF = deltaPitch * 0.5f * boostFactor;
    float yawFF   = deltaYaw   * 0.5f * boostFactor;
    
    lastRcRoll = rcRoll; lastRcPitch = rcPitch; lastRcYaw = rcYaw;
    
    /* ==================== [Optimal-C/D] 分频控制 ==================== */
    /* Angle PID: 500Hz (每4次跑1次)
     * Rate PID:  2kHz  (每次都跑)
     * Euler计算:  500Hz (与Angle同步)
     */
    static uint8_t angleDivider = 0;
    static float targetRollRate = 0.0f;
    static float targetPitchRate = 0.0f;
    extern uint32_t angleLoopCount;  // 引用全局计数器
    
    angleDivider++;
    if (angleDivider >= 4) {  // 2000Hz / 4 = 500Hz
        angleDivider = 0;
        angleLoopCount++;
        
        /* [D] 计算欧拉角 (500Hz, 包含 atan2f/asinf) */
        Attitude_ComputeEuler();
        Attitude_GetEulerCached(&attitude);
        
        /* [C] Angle PID (外环, 500Hz) */
        float targetRoll  = (float)rcRoll  * (MAX_ANGLE / 500.0f);  // RC: -500~500 -> 角度
        float targetPitch = (float)rcPitch * (MAX_ANGLE / 500.0f);
        
        /* 限幅 */
        if (targetRoll > MAX_ANGLE) targetRoll = MAX_ANGLE;
        if (targetRoll < -MAX_ANGLE) targetRoll = -MAX_ANGLE;
        if (targetPitch > MAX_ANGLE) targetPitch = MAX_ANGLE;
        if (targetPitch < -MAX_ANGLE) targetPitch = -MAX_ANGLE;
        
        /* Angle -> Rate */
        targetRollRate  = PID_Calculate(&flightPID.roll_angle,  targetRoll,  attitude.roll,  dt * 4.0f);
        targetPitchRate = PID_Calculate(&flightPID.pitch_angle, targetPitch, attitude.pitch, dt * 4.0f);
    } else {
        /* 非 Angle 帧：使用缓存的欧拉角 (无计算) */
        Attitude_GetEulerCached(&attitude);
    }
    
    /* [C] Rate PID (内环, 2kHz) - 每次都跑 */
    float targetYawRate = (float)rcYaw * 0.5f;  // Yaw 保持 rate 模式
    
    float rollOutput  = PID_Calculate(&flightPID.roll_rate,  targetRollRate,  imu->gyro[0], dt) + rollFF;
    float pitchOutput = PID_Calculate(&flightPID.pitch_rate, targetPitchRate, imu->gyro[1], dt) + pitchFF;
    float yawOutput   = PID_Calculate(&flightPID.yaw_rate,   targetYawRate,   imu->gyro[2], dt) + yawFF;
    
    /* 恢复 PID 参数 (TPA + Anti-Gravity) */
    flightPID.roll_rate.Kp = orig_Kp_roll; flightPID.roll_rate.Kd = orig_Kd_roll;
    flightPID.pitch_rate.Kp = orig_Kp_pitch; flightPID.pitch_rate.Kd = orig_Kd_pitch;
    flightPID.roll_rate.Ki = orig_Ki_roll;
    flightPID.pitch_rate.Ki = orig_Ki_pitch;
    
    /* 积分清零逻辑 */
    if (rcThrottle < 100) {
        PID_ResetIntegral(&flightPID.roll_rate);
        PID_ResetIntegral(&flightPID.pitch_rate);
        PID_ResetIntegral(&flightPID.yaw_rate);
    }
    
    /* 混控与输出 */
    /* 混控与输出 */
    int16_t finalThrottle = 0;
    if (rcThrottle > 40) {
        finalThrottle = rcThrottle;
        
        /* [AltHold] 叠加定高输出 */
        if (AltHold_IsEnabled()) {
            finalThrottle += AltHold_GetOutput();
        }
        
        /* 基础混控 */
        // 为了防止定高叠加导致溢出，Motor_Mix 内部会限幅
        // 但这里我们简单处理基准油门: 如果定高介入，用户油门作为Base
    }
    else {
        finalThrottle = 0;
        rollOutput = 0; pitchOutput = 0; yawOutput = 0;
        // 油门极低时复位定高只是为了保险，实际逻辑在主循环控制 Enable/Disable
    }
    
    /* ==================== [Phase-1.3] Throttle Boost ==================== */
    /* 油门瞬变时增加电机输出，补偿电机/电调延迟 (~5ms) */
    static float lastThrottleBoost = 0;
    float throttleChangeRate = fabsf((float)rcThrottle - (float)lastThrottle); // lastThrottle来自Anti-Gravity
    float throttleBoost = 2.0f * throttleChangeRate;  // kBoost = 2.0
    
    /* 平滑衰减 */
    throttleBoost = throttleBoost * 0.8f + lastThrottleBoost * 0.2f;
    lastThrottleBoost = throttleBoost;

    Motor_Mix(finalThrottle, (int16_t)rollOutput, (int16_t)pitchOutput, -(int16_t)yawOutput, &motorOutput);
    
    /* ==================== VBat 电压补偿 ==================== */
    /* 电池电压下降时，推力会减弱；补偿后手感更一致 */
    #define VBAT_FULL      12.6f   // 满电电压
    #define VBAT_NOMINAL   11.1f   // 标称电压
    #define VBAT_COMP_MAX  1.15f   // 最大补偿系数 (15%)
    
    float vbat = Battery_GetVoltage();
    float vbatComp = 1.0f;
    
    if (vbat > 8.0f && vbat < VBAT_FULL) {  // 有效电压范围
        vbatComp = VBAT_NOMINAL / vbat;
        if (vbatComp > VBAT_COMP_MAX) vbatComp = VBAT_COMP_MAX;
        if (vbatComp < 1.0f) vbatComp = 1.0f;
    }
    
    /* 应用补偿 (仅对油门部分) + Throttle Boost + Dynamic Idle */
    for (int i = 0; i < 4; i++) {
        int32_t pwm = motorOutput.motor[i];
        int32_t throttlePart = pwm - MOTOR_PWM_MIN;
        throttlePart = (int32_t)(throttlePart * vbatComp);
        pwm = MOTOR_PWM_MIN + throttlePart;
        
        /* [Phase-1.3] 叠加Throttle Boost */
        pwm += (int32_t)throttleBoost;
        
        /* [Phase-2.1] 叠加Dynamic Idle */
        pwm += (int32_t)motorIdleOffset;
        
        if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
        motorOutput.motor[i] = (uint16_t)pwm;
    }
    
    /* PWM 输出 (如果启用了 DShot 会自动切换) */
    if (USE_DSHOT) {
        DShot_WriteAllThrottle(
            DShot_PWMToThrottle(motorOutput.motor[0]),
            DShot_PWMToThrottle(motorOutput.motor[1]),
            DShot_PWMToThrottle(motorOutput.motor[2]),
            DShot_PWMToThrottle(motorOutput.motor[3])
        );
        /* DShot Update在主循环调用 */
    }
    /* ==================== [PWM Sync] 设置待更新PWM值 ==================== */
    /* 由TIM1中断在下个400Hz周期时原子更新，消除抖动 */
    Motor_SetPendingPWM(motorOutput.motor[0], motorOutput.motor[1],
                        motorOutput.motor[2], motorOutput.motor[3]);
}

/**
 * @brief 初始化LED
 */
static void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
 * @brief 切换LED
 */
static void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/**
 * @brief 初始化独立看门狗 (IWDG)
 * @note  超时时间 = (Prescaler * Reload) / 32000
 *        Prescaler=32 -> 1ms/tick
 *        Reload=100 -> 100ms
 */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;  // 32kHz / 32 = 1kHz
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 500;                   // 500ms (Relaxed for I2C blocking)
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        /* Initialization Error */
        // 不要卡死在这里，否则看门狗可能会直接复位
    }
}

/* ==================== 2kHz DRDY 中断回调 ==================== */

/**
 * @brief DRDY 中断回调 - 2kHz 控制循环
 * @note  由 drdy_imu.c 在 SPI DMA 完成后调用
 */
void DRDY_FlightControl_Callback(DRDY_IMU_Data_t *data, float dt)
{
    /* ==================== 统一电机控制入口 ==================== */
    /* 只有这个 ISR 上下文允许写电机，防止竞态冲突 */
    
    if (!armed) {
        /* 未解锁状态：强制维持最小油门 (Idle / Disarmed) */
        Motor_SetAllPWM(MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN);
        
        /* 也可以做一些后台校准，但不要驱动电机 */
        return;
    }
    
    /* 解锁状态：执行飞行控制算法 */
    /* 
     * 注意: Attitude_Update 已经在 drdy_imu.c 中通过 Attitude_Update 调用了吗？
     * AGENT CHECK: Attitude_Update logic inside ISR
     * 刚才我看到 main.c 原代码里 Attitude_Update 就在这个 Callback 里 (Line 563)
     * 所以这里保持原样，只是把之后的 FlightControl(dt) 加上
     */
    
    /* 使用 DRDY 提供的滤波后数据 */
    float gx = data->gyro[0] * DEG_TO_RAD;
    float gy = data->gyro[1] * DEG_TO_RAD;
    float gz = data->gyro[2] * DEG_TO_RAD;
    
    /* 姿态解算 @ 2kHz */
    Attitude_Update(gx, gy, gz, data->accel[0], data->accel[1], data->accel[2]);
    Attitude_GetEuler(&attitude);
    
    /* [Phase-3] 定高预测步骤已移至 FlightControl 内部统一处理 */
    // AltHold_Predict(accZ_G, dt);
    
    /* 运行全功能飞控 (PID + Mixer) */
    FlightControl(data, dt);
    
}
