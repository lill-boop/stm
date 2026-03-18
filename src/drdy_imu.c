/**
 * @file drdy_imu.c
 * @brief DRDY 中断驱动 IMU 实现
 * @note  2kHz EXTI 中断 + SPI DMA Burst 读取
 */

#include "drdy_imu.h"
#include "stm32g4xx_hal.h"
#include "bsp_spi.h"
#include "icm42688.h"
#include <string.h>
#include <math.h>

/* ==================== DWT 计时器 ==================== */
#define DWT_CONTROL  (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT   (*(volatile uint32_t*)0xE0001004)
#define DWT_LAR      (*(volatile uint32_t*)0xE0001FB0)
#define SCB_DEMCR    (*(volatile uint32_t*)0xE000EDFC)
#define CPU_FREQ_MHZ 170

/* ==================== 滤波器系数 ==================== */
/* Notch Filter: 170Hz @ 2kHz, Q=2.1 */
#define NOTCH_B0   0.953f
#define NOTCH_B1  -1.618f
#define NOTCH_B2   0.953f
#define NOTCH_A1  -1.618f
#define NOTCH_A2   0.906f

/* PT2 Lowpass: 90Hz @ 2kHz */
#define LPF_COEFF  0.25f

/* ==================== 私有变量 ==================== */
/* 当前 Notch 系数 (可运行时更新) */
static float notch_b0 = 0.953f;
static float notch_b1 = -1.618f;
static float notch_b2 = 0.953f;
static float notch_a1 = -1.618f;
static float notch_a2 = 0.906f;

static SPI_HandleTypeDef hspi1_dma;
static DMA_HandleTypeDef hdma_spi1_rx;
static DMA_HandleTypeDef hdma_spi1_tx;

/* DMA Buffers: 必须在 SRAM1/SRAM2 (STM32G4 不能用 CCMRAM 做 DMA) */
/* 未加特殊段属性，默认在 SRAM，安全。 */
static uint8_t spiTxBuf[16];
static uint8_t spiRxBuf[16];
static volatile uint8_t spiDmaComplete = 0;

static DRDY_IMU_Data_t imuData;
DRDY_Stats_t stats;  // Export for p99 calc

static uint32_t lastTimestamp = 0;
static uint8_t pwmDivider = 0;
static volatile uint8_t updatePwmFlag = 0;

/* 滤波器状态 */
static float notchZ1[3] = {0}, notchZ2[3] = {0};
static float notch2Z1[3] = {0}, notch2Z2[3] = {0};  // Notch2 状态
static float lpfState0[3] = {0}, lpfState1[3] = {0};

/* 校准偏移 & 映射 */
// 默认映射保持原代码逻辑: X=-x, Y=-y, Z=z
static uint8_t axisMap[3] = {0, 1, 2};
static int8_t axisSign[3] = {-1, -1, 1};
static float gyroOffset[3] = {0}; // Body Frame Bias

static float gyroSensitivity = 16.4f;
static float accelSensitivity = 2048.0f;

/* 调试数据 */
volatile IMU_Debug_Data_t imuDebugData;
bool imuDebugActive = false;

/* [Optimal] ISR 性能监控 - 环形缓冲区 */
#define ISR_BUFFER_SIZE 2048
uint16_t isr_us_buffer[ISR_BUFFER_SIZE];  // Export for p99 calc
static uint16_t isr_buffer_idx = 0;

/* [Optimal] 异常帧计数 */
static uint8_t consecutive_bad_dt = 0;
#define MAX_CONSECUTIVE_BAD_DT 10

/* [Optimal] 上一帧电机输出缓存 (用于 bad frame 冻结) */
static uint16_t last_motor_pwm[4] = {1000, 1000, 1000, 1000};

/* [Optimal-G] Gyro 环形缓冲区 (用于 FFT/Goertzel) */
#define GYRO_BUFFER_SIZE 256
static float gyro_ring_buffer[GYRO_BUFFER_SIZE];
static uint16_t gyro_buffer_head = 0;

/* ==================== 私有函数声明 ==================== */
static void DWT_Init(void);
static uint32_t DWT_GetCycles(void);
static void EXTI_Init(void);
static void SPI_DMA_Init(void);
static void ConfigureICM42688_DRDY(void);
static void ProcessIMUData(void);

/* ==================== API 实现 ==================== */

void DRDY_IMU_Init(void)
{
    /* 初始化 DWT 计时器 */
    DWT_Init();
    
    /* 清零统计 */
    memset(&stats, 0, sizeof(stats));
    stats.minLoopTime_us = 0xFFFFFFFF;
    
    /* 配置 ICM42688 输出 DRDY 到 INT1 */
    ConfigureICM42688_DRDY();
    
    /* 初始化 SPI DMA */
    SPI_DMA_Init();
    
    /* 初始化 EXTI (PB1) */
    EXTI_Init();
    
    /* 准备读取命令 (读取 TEMP + ACCEL + GYRO) */
    spiTxBuf[0] = ICM42688_TEMP_DATA1 | 0x80;  // 读取起始地址
    for (int i = 1; i < 15; i++) spiTxBuf[i] = 0xFF;
}

void DRDY_IMU_Start(void)
{
    HAL_NVIC_EnableIRQ(DRDY_EXTI_IRQn);
}

void DRDY_IMU_Stop(void)
{
    HAL_NVIC_DisableIRQ(DRDY_EXTI_IRQn);
}

DRDY_IMU_Data_t* DRDY_IMU_GetData(void)
{
    return &imuData;
}

DRDY_Stats_t* DRDY_IMU_GetStats(void)
{
    return &stats;
}

bool DRDY_IMU_ShouldUpdatePWM(void)
{
    if (updatePwmFlag) {
        updatePwmFlag = 0;
        return true;
    }
    return false;
}

/* ==================== 中断处理 ==================== */

/* ==================== 中断处理 ==================== */

void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        
        /* 严谨的时序监控: 如果上一帧尚未完成 (DMA忙或计算未完) */
        /* 注意：hspi1_dma.State 在传输完成回调前一直为 BUSY */
        if (hspi1_dma.State != HAL_SPI_STATE_READY) {
            stats.missed_samples++;
            return; // 放弃这一帧，保证不重入
        }

        /* 启动 SPI DMA 读取 */
        ICM42688_CS_LOW();
        
        /* 等待 CS 稳定 (面包板/杜邦线信号建立时间) */
        /* 等待 CS 稳定 (杜邦线建立时间 ~50ns) */
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        
        if (HAL_SPI_TransmitReceive_DMA(&hspi1_dma, spiTxBuf, spiRxBuf, 15) != HAL_OK) {
            stats.missed_samples++;
            ICM42688_CS_HIGH();
        }
    }
}

/**
 * @brief SPI DMA 完成回调 (DRDY ISR 核心)
 * @note [Optimal] Bad dt 检测 + ISR 测量 + 环形缓冲
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1_dma) {
        static volatile uint8_t isRunning = 0;
        if (isRunning) return;
        isRunning = 1;

        ICM42688_CS_HIGH();
        
        uint32_t now = DWT_GetCycles();
        uint32_t cycles = now - lastTimestamp;
        float dt = (float)cycles / (CPU_FREQ_MHZ * 1000000.0f);
        
        /* [B] dt 异常检测 */
        uint8_t bad_dt_flag = 0;
        if (dt < 0.0002f || dt > 0.002f) {
            bad_dt_flag = 1;
            stats.bad_dt_count++;
            consecutive_bad_dt++;
            if (consecutive_bad_dt > MAX_CONSECUTIVE_BAD_DT) {
                consecutive_bad_dt = MAX_CONSECUTIVE_BAD_DT;
            }
        } else {
            consecutive_bad_dt = 0;
        }
        
        lastTimestamp = now;
        imuData.dt = dt;
        imuData.timestamp = now / CPU_FREQ_MHZ;
        
        /* dt 统计 */
        uint32_t loopTime_us = cycles / CPU_FREQ_MHZ;
        if (loopTime_us > stats.maxLoopTime_us) stats.maxLoopTime_us = loopTime_us;
        if (loopTime_us < stats.minLoopTime_us) stats.minLoopTime_us = loopTime_us;
        stats.avgLoopTime_us = (stats.avgLoopTime_us * 127 + loopTime_us) >> 7;
        stats.sampleCount++;
        
        ProcessIMUData();
        
        if (++pwmDivider >= PWM_DIVIDER) {
            pwmDivider = 0;
            updatePwmFlag = 1;
        }
        
        /* [B] Bad frame 冻结 */
        if (bad_dt_flag) {
            isRunning = 0;
            return;
        }
        
        /* [A] ISR 测量 */
        uint32_t execStart = DWT_CYCCNT;
        DRDY_FlightControl_Callback(&imuData, dt);
        uint32_t execEnd = DWT_CYCCNT;
        
        uint32_t execTime_us = (execEnd - execStart) / CPU_FREQ_MHZ;
        if (execTime_us > stats.maxExecTime_us) stats.maxExecTime_us = execTime_us;
        stats.avgExecTime_us = (stats.avgExecTime_us * 127 + execTime_us) >> 7;
        
        /* 环形缓冲区记录 ISR 耗时 */
        isr_us_buffer[isr_buffer_idx] = (uint16_t)execTime_us;
        isr_buffer_idx = (isr_buffer_idx + 1) & (ISR_BUFFER_SIZE - 1);
        
        /* [Optimal-G] 记录 Gyro 数据 (取 Roll 轴，能量通常最大) */
        gyro_ring_buffer[gyro_buffer_head] = imuData.gyro[0];
        gyro_buffer_head = (gyro_buffer_head + 1) & (GYRO_BUFFER_SIZE - 1);
        
        if (execTime_us > 100) stats.isrOverbudget++;
        
        isRunning = 0;
    }
}


/* ==================== 私有函数实现 ==================== */

static void DWT_Init(void)
{
    SCB_DEMCR |= 0x01000000;
    DWT_LAR = 0xC5ACCE55;
    DWT_CYCCNT = 0;
    DWT_CONTROL |= 1;
}

static uint32_t DWT_GetCycles(void)
{
    return DWT_CYCCNT;
}

static void SPI_DMA_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    
    /* SPI1 配置 */
    hspi1_dma.Instance = SPI1;
    hspi1_dma.Init.Mode = SPI_MODE_MASTER;
    hspi1_dma.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1_dma.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1_dma.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1_dma.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1_dma.Init.NSS = SPI_NSS_SOFT;
    /* 超低速: 170MHz / 128 = 1.3MHz (面包板/杜邦线稳定性优化) */
    hspi1_dma.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1_dma.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1_dma.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1_dma.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1_dma.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    
    /* DMA RX */
    hdma_spi1_rx.Instance = DMA1_Channel2;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi1_rx);
    __HAL_LINKDMA(&hspi1_dma, hdmarx, hdma_spi1_rx);
    
    /* DMA TX */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_spi1_tx);
    __HAL_LINKDMA(&hspi1_dma, hdmatx, hdma_spi1_tx);
    
    HAL_SPI_Init(&hspi1_dma);
    
    /* DMA 中断优先级 - 设为 1 (次高)，允许被 EXTI(0) 抢占 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0); // RX Complete (Control Loop Entry)
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0); // TX Complete
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

static void EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);  // 最高优先级 (0)，保证 DRDY 准时响应
}

static void ConfigureICM42688_DRDY(void)
{
    /* 配置 INT1 输出 DRDY */
    ICM42688_WriteReg(ICM42688_INT_CONFIG, 0x02);   // INT1 push-pull, active high
    ICM42688_WriteReg(ICM42688_INT_SOURCE0, 0x08);  // UI_DRDY_INT1_EN
    
    /* 配置 2kHz 输出数据率 */
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG0, 0x05);   // 2000dps, 2kHz ODR
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, 0x05);  // 16g, 2kHz ODR
}

static void ProcessIMUData(void)
{
    /* 解析 DMA 数据 */
    int16_t temp_raw = (int16_t)((spiRxBuf[1] << 8) | spiRxBuf[2]);
    int16_t raw_acc[3], raw_gyro[3];
    
    raw_acc[0] = (int16_t)((spiRxBuf[3] << 8) | spiRxBuf[4]);
    raw_acc[1] = (int16_t)((spiRxBuf[5] << 8) | spiRxBuf[6]);
    raw_acc[2] = (int16_t)((spiRxBuf[7] << 8) | spiRxBuf[8]);
    
    raw_gyro[0] = (int16_t)((spiRxBuf[9] << 8) | spiRxBuf[10]);
    raw_gyro[1] = (int16_t)((spiRxBuf[11] << 8) | spiRxBuf[12]);
    raw_gyro[2] = (int16_t)((spiRxBuf[13] << 8) | spiRxBuf[14]);
    
    /* 1. 转换为物理单位 (Sensor Frame) & 温度 */
    imuData.temperature = ((float)temp_raw / 132.48f) + 25.0f;
    
    // 简单预检：检测异常数据
    // 1. 任一轴超过 3g = 坏帧
    // 2. 合加速度太小 (< 0.5g) = 可能全零/无效 = 坏帧
    // 3. 合加速度太大 (> 2g) = 可能在剧烈运动或数据错误
    
    // 粗略转 float gCheck (为了判断)
    float ax_g = (float)raw_acc[0] / accelSensitivity;
    float ay_g = (float)raw_acc[1] / accelSensitivity;
    float az_g = (float)raw_acc[2] / accelSensitivity;
    
    // 计算合加速度
    float accel_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    
    int isBadFrame = 0;
    // 任一轴超过 3g
    if (fabsf(ax_g) > 3.0f || fabsf(ay_g) > 3.0f || fabsf(az_g) > 3.0f) isBadFrame = 1;
    // 合加速度太小 (可能是全零数据)
    if (accel_mag < 0.5f) isBadFrame = 2;
    // 合加速度太大 (静态时应该约 1g)
    if (accel_mag > 4.0f) isBadFrame = 3;
    
    if (isBadFrame) {
        stats.spiErrors++;
        stats.missed_samples++; // 视为丢帧
        
        /* 陷阱：打印 Hex Dump (仅打印前5次避免刷屏，或低频打印) */
        static int printCount = 0;
        if (printCount < 5 || (stats.spiErrors % 50 == 0)) {
            // 需要 include <stdio.h> 在上面，或者使用 BSP_UART_Printf 的声明
            extern void BSP_UART_Printf(const char *fmt, ...);
            /*
            BSP_UART_Printf("\r\n[BAD SPI] A:%.2f %.2f %.2f | Hex:", ax_g, ay_g, az_g);
            for(int k=0; k<16; k++) {
                BSP_UART_Printf(" %02X", spiRxBuf[k]);
            }
            BSP_UART_Printf("\r\n");
            */
            printCount++;
        }
        return; // 直接丢弃，不更新 imuData
    }

    float acc_sensor[3], gyro_sensor[3];
    for(int i=0; i<3; i++) {
        acc_sensor[i] = (float)raw_acc[i] / accelSensitivity;
        gyro_sensor[i] = (float)raw_gyro[i] / gyroSensitivity;
    }

    /* 2. 映射到机体坐标系 (Body Frame) */
    /* 应用 Axis Map & Sign */
    float acc_body[3], gyro_body[3];
    for(int i=0; i<3; i++) {
        acc_body[i] = (float)axisSign[i] * acc_sensor[axisMap[i]];
        // Gyro: Map -> Subtract Bias
        float val = (float)axisSign[i] * gyro_sensor[axisMap[i]];
        gyro_body[i] = val - gyroOffset[i];
    }
    
    // 保存 Accel
    imuData.accel[0] = acc_body[0];
    imuData.accel[1] = acc_body[1];
    imuData.accel[2] = acc_body[2];
    
    // 保存 RAW Gyro (用于Blackbox/Debug, 此时已去偏)
    imuData.gyro_raw[0] = gyro_body[0];
    imuData.gyro_raw[1] = gyro_body[1];
    imuData.gyro_raw[2] = gyro_body[2];
    
    /* 3. Gyro 滤波 (Notch -> PT2) */
    for (int i = 0; i < 3; i++) {
        float in = gyro_body[i];
        
        // Notch
        float out = notch_b0 * in + notchZ1[i];
        notchZ1[i] = notch_b1 * in - notch_a1 * out + notchZ2[i];
        notchZ2[i] = notch_b2 * in - notch_a2 * out;
        
        // PT2 LPF
        lpfState0[i] += LPF_COEFF * (out - lpfState0[i]);
        lpfState1[i] += LPF_COEFF * (lpfState0[i] - lpfState1[i]);
        
        imuData.gyro[i] = lpfState1[i];
    }
    
    /* 4. 更新调试数据 (如果激活) */
    if (imuDebugActive) {
        imuDebugData.dt_s = imuData.dt;
        imuDebugData.gyro_dps[0] = imuData.gyro[0];
        imuDebugData.gyro_dps[1] = imuData.gyro[1];
        imuDebugData.gyro_dps[2] = imuData.gyro[2];
        imuDebugData.acc_g[0] = imuData.accel[0];
        imuDebugData.acc_g[1] = imuData.accel[1];
        imuDebugData.acc_g[2] = imuData.accel[2];
        
        // 计算 Norm
        float a_sq = acc_body[0]*acc_body[0] + acc_body[1]*acc_body[1] + acc_body[2]*acc_body[2];
        imuDebugData.acc_norm = sqrtf(a_sq);
        
        // 简单模拟 Attitude 的 Gating Weight (仅参考)
        float err = fabsf(imuDebugData.acc_norm - 1.0f);
        if (err < 0.1f) imuDebugData.acc_weight = 1.0f;
        else if (err > 0.4f) imuDebugData.acc_weight = 0.0f;
        else imuDebugData.acc_weight = 1.0f - (err - 0.1f) / 0.3f;
    }
}

/* ... existing IRQ handlers ... */
/* 插入在 IRQ handler 之后, SetNotchFreq 之前 */

/* ==================== 配置 API 实现 ==================== */

void DRDY_IMU_SetAxisMapping(uint8_t map[3], int8_t sign[3])
{
    for(int i=0; i<3; i++) {
        axisMap[i] = map[i];
        axisSign[i] = sign[i];
    }
}

void DRDY_IMU_GetAxisMapping(uint8_t map[3], int8_t sign[3])
{
    for(int i=0; i<3; i++) {
        map[i] = axisMap[i];
        sign[i] = axisSign[i];
    }
}

void DRDY_IMU_SetGyroBias(float bias[3])
{
    for(int i=0; i<3; i++) gyroOffset[i] = bias[i];
}

void DRDY_IMU_GetGyroBias(float bias[3])
{
    for(int i=0; i<3; i++) bias[i] = gyroOffset[i];
}

void DRDY_IMU_CalibrateGyro(void)
{
    float sum[3] = {0};
    int samples = 0;
    
    /* 简单的阻塞式累积 (约2秒) */
    /* 注意：主循环调用此函数时，ISR 仍在后台运行并更新 imuData */
    
    HAL_Delay(100); // 等待稳定
    
    for(int i=0; i<200; i++) { // 200 * 10ms = 2000ms
        HAL_Delay(10);
        
        /* 还原未去偏的 Body Frame 数据 */
        // raw = bias_removed + bias
        // 这里用 gyro (filtered) 比较稳
        sum[0] += imuData.gyro[0] + gyroOffset[0];
        sum[1] += imuData.gyro[1] + gyroOffset[1];
        sum[2] += imuData.gyro[2] + gyroOffset[2];
        samples++;
    }
    
    if (samples > 0) {
        gyroOffset[0] = sum[0] / samples;
        gyroOffset[1] = sum[1] / samples;
        gyroOffset[2] = sum[2] / samples;
    }
}

/* DMA 中断处理 */
void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/* ==================== 默认弱回调 ==================== */
__attribute__((weak)) void DRDY_FlightControl_Callback(DRDY_IMU_Data_t *data, float dt)
{
    /* 用户需要在 main.c 中实现此函数 */
    (void)data;
    (void)dt;
}

/**
 * @brief [Optimal-G] 获取 Gyro 环形缓冲区
 */
void DRDY_IMU_GetGyroBuffer(uint16_t count, float *buffer)
{
    if (count > GYRO_BUFFER_SIZE) count = GYRO_BUFFER_SIZE;
    
    /* 简单的非原子拷贝 (对于 50Hz 分析，极少量撕裂可接受) */
    /* 从最新的数据倒推拷贝 count 个 */
    uint16_t head = gyro_buffer_head;
    for (int i = 0; i < count; i++) {
        int idx = head - 1 - i;
        if (idx < 0) idx += GYRO_BUFFER_SIZE;
        buffer[i] = gyro_ring_buffer[idx];
    }
}

/* ==================== 动态 Notch 调参 ==================== */

/* 旧版单参数 API 已移除，使用 DRDY_IMU_SetNotchFreq(freq, Q) */

/**
 * @brief 设置 Notch1 滤波器 (API 升级: 支持 Q 参数)
 */
void DRDY_IMU_SetNotchFreq(float freq, float q)
{
    if (freq < 50.0f) freq = 50.0f;
    if (freq > 500.0f) freq = 500.0f;
    if (q < 0.5f) q = 0.5f;
    
    float sampleRate = 2000.0f;
    float omega = 2.0f * 3.14159f * freq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q);
    
    float a0 = 1.0f + alpha;
    
    notch_b0 = 1.0f / a0;
    notch_b1 = -2.0f * cs / a0;
    notch_b2 = 1.0f / a0;
    notch_a1 = -2.0f * cs / a0;
    notch_a2 = (1.0f - alpha) / a0;
}

/* Notch2 系数 */
static float notch2_b0 = 1.0f, notch2_b1 = 0.0f, notch2_b2 = 0.0f;
static float notch2_a1 = 0.0f, notch2_a2 = 0.0f;

/**
 * @brief 设置 Notch2 滤波器 (FFT 双 Notch 用)
 */
void DRDY_IMU_SetNotchFreq2(float freq, float q)
{
    if (freq < 50.0f) freq = 50.0f;
    if (freq > 500.0f) freq = 500.0f;
    if (q < 0.5f) q = 0.5f;
    
    float sampleRate = 2000.0f;
    float omega = 2.0f * 3.14159f * freq / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q);
    
    float a0 = 1.0f + alpha;
    
    notch2_b0 = 1.0f / a0;
    notch2_b1 = -2.0f * cs / a0;
    notch2_b2 = 1.0f / a0;
    notch2_a1 = -2.0f * cs / a0;
    notch2_a2 = (1.0f - alpha) / a0;
}
 
 /* ==================== [Phase-2.2] Dynamic Gyro LPF ==================== */
static float current_gyro_lpf_cutoff = 200.0f;
void DRDY_IMU_SetDynamicGyroLPF(float throttlePercent) {
    float target_cutoff = 200.0f - throttlePercent * 1.0f;
    if (target_cutoff < 80.0f) target_cutoff = 80.0f;
    if (target_cutoff > 250.0f) target_cutoff = 250.0f;
    current_gyro_lpf_cutoff = current_gyro_lpf_cutoff * 0.95f + target_cutoff * 0.05f;
}
