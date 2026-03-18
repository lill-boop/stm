/**
 * @file ms5611.c
 * @brief MS5611 气压传感器驱动实现
 * @note  使用状态机模式，非阻塞读取
 * 
 * MS5611 是一款高精度气压计，分辨率可达 10cm 高度
 * 转换流程: 发送命令 -> 等待转换 -> 读取结果
 */

#include "ms5611.h"
#include "bsp_i2c.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ==================== 寄存器定义 ==================== */
#define CMD_RESET       0x1E
#define CMD_PROM_READ   0xA0    // A0-AE: PROM (校准系数)
#define CMD_CONV_D1     0x40    // 转换压力 (D1)
#define CMD_CONV_D2     0x50    // 转换温度 (D2)
#define CMD_ADC_READ    0x00

/* ==================== 私有变量 ==================== */
static uint16_t C[8];           // 校准系数 C1-C6 (C0, C7 unused)
static uint32_t D1, D2;         // ADC 原始值
static MS5611_Data_t data;
static float groundPressure = 101325.0f;  // 基准气压 (Pa)
static float lastAltitude = 0.0f;
static uint32_t lastUpdateTime = 0;

/* 状态机 */
typedef enum {
    STATE_IDLE,
    STATE_CONV_TEMP,    // 等待温度转换
    STATE_CONV_PRESS,   // 等待气压转换
} MS5611_State_t;

static MS5611_State_t state = STATE_IDLE;
static uint32_t convStartTime = 0;
static MS5611_OSR_t osr = MS5611_OSR_256;  // 降低精度以测试稳定性

/* 转换等待时间 (ms) */
static const uint8_t convDelays[] = {1, 2, 3, 5, 10};

/* ==================== 私有函数 ==================== */
static uint32_t ReadADC(uint8_t addr);
static void Calculate(void);
static float PressureToAltitude(float pressure);

/* ==================== API 实现 ==================== */

bool MS5611_Init(void)
{
    extern void BSP_UART_Printf(const char *fmt, ...);
    
    /* 初始化 I2C */
    BSP_I2C1_Init();
    
    HAL_Delay(50);  // 等待模块稳定
    
    /* 尝试两个可能的地址 */
    uint8_t addr = MS5611_I2C_ADDR;  // 默认 0x77
    
    BSP_UART_Printf("[BARO] Trying address 0x%02X...\r\n", addr);
    if (!BSP_I2C1_WriteCmd(addr, CMD_RESET)) {
        /* 尝试另一个地址 */
        addr = (addr == 0x77) ? 0x76 : 0x77;
        BSP_UART_Printf("[BARO] Trying address 0x%02X...\r\n", addr);
        if (!BSP_I2C1_WriteCmd(addr, CMD_RESET)) {
            BSP_UART_Printf("[BARO] No device found!\r\n");
            return false;
        }
    }
    BSP_UART_Printf("[BARO] Device found at 0x%02X!\r\n", addr);
    
    HAL_Delay(10);  // 复位需要 2.8ms
    
    /* 读取校准系数 (PROM) */
    uint8_t buf[2];
    for (int i = 0; i < 8; i++) {
        if (!BSP_I2C1_Read(addr, CMD_PROM_READ + i * 2, buf, 2)) {
            BSP_UART_Printf("[BARO] PROM read failed at C%d!\r\n", i);
            return false;
        }
        C[i] = (buf[0] << 8) | buf[1];
    }
    
    /* 打印校准系数用于调试 */
    BSP_UART_Printf("[BARO] PROM: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\r\n",
        C[1], C[2], C[3], C[4], C[5], C[6]);
    
    /* 校验系数有效性 */
    if (C[1] == 0 || C[1] == 0xFFFF || C[2] == 0 || C[2] == 0xFFFF) {
        BSP_UART_Printf("[BARO] Invalid PROM data!\r\n");
        return false;
    }
    
    /* 初始化数据 */
    data.valid = false;
    data.pressure = 101325.0f;
    data.temperature = 25.0f;
    data.altitude = 0.0f;
    data.altitudeRaw = 0.0f;
    data.verticalSpeed = 0.0f;
    
    /* 预读几次以稳定基准 - 增加延时，等待模块稳定 */
    HAL_Delay(50);  // 额外等待模块稳定
    
    int validReadings = 0;
    for (int i = 0; i < 10 && validReadings < 3; i++) {
        /* 温度转换 */
        if (!BSP_I2C1_WriteCmd(addr, CMD_CONV_D2 | osr)) {
            HAL_Delay(20);
            continue;
        }
        HAL_Delay(15);  // OSR_4096 需要约 9ms
        
        /* 读取温度 ADC */
        D2 = ReadADC(addr);
        if (D2 == 0) {
            HAL_Delay(20);
            continue;
        }
        
        /* 气压转换 */
        if (!BSP_I2C1_WriteCmd(addr, CMD_CONV_D1 | osr)) {
            HAL_Delay(20);
            continue;
        }
        HAL_Delay(15);
        
        /* 读取气压 ADC */
        D1 = ReadADC(addr);
        if (D1 == 0) {
            HAL_Delay(20);
            continue;
        }
        
        Calculate();
        validReadings++;
        
        /* 检查计算结果是否合理 (放宽到 30000 以供调试) */
        if (data.pressure < 30000 || data.pressure > 120000) {
            BSP_UART_Printf("[BARO] Pressure out of range: %ld Pa\r\n", (int32_t)data.pressure);
            validReadings = 0;  // 重置计数
            continue;
        }
    }
    
    /* 打印初始读数 */
    if (validReadings > 0) {
        BSP_UART_Printf("[BARO] Initial: P=%ldPa T=%ldC (D1=0x%06lX D2=0x%06lX)\r\n", 
            (int32_t)data.pressure, (int32_t)data.temperature, D1, D2);
    } else {
        BSP_UART_Printf("[BARO] Warning: No valid readings during init!\r\n");
        data.pressure = 101325.0f;  // 使用标准大气压
        data.temperature = 25.0f;
    }
    
    /* 设置基准气压 */
    groundPressure = data.pressure;
    data.altitude = 0.0f;
    data.valid = true;
    lastAltitude = 0.0f;
    lastUpdateTime = HAL_GetTick();
    
    state = STATE_IDLE;
    
    return true;
}

void MS5611_StartConversion(void)
{
    if (state == STATE_IDLE) {
        /* 开始温度转换 */
        BSP_I2C1_WriteCmd(MS5611_I2C_ADDR, CMD_CONV_D2 | osr);
        convStartTime = HAL_GetTick();
        state = STATE_CONV_TEMP;
    }
}

bool MS5611_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t delay = convDelays[osr >> 1];
    
    switch (state) {
        case STATE_IDLE:
            /* 开始温度转换 */
            BSP_I2C1_WriteCmd(MS5611_I2C_ADDR, CMD_CONV_D2 | osr);
            convStartTime = now;
            state = STATE_CONV_TEMP;
            return false;
        
        case STATE_CONV_TEMP:
            if (now - convStartTime >= delay) {
                /* 读取温度 */
                D2 = ReadADC(MS5611_I2C_ADDR);
                
                /* 开始气压转换 */
                BSP_I2C1_WriteCmd(MS5611_I2C_ADDR, CMD_CONV_D1 | osr);
                convStartTime = now;
                state = STATE_CONV_PRESS;
            }
            return false;
        
        case STATE_CONV_PRESS:
            if (now - convStartTime >= delay) {
                /* 读取气压 */
                D1 = ReadADC(MS5611_I2C_ADDR);
                
                /* 计算温度补偿后的气压和高度 */
                Calculate();
                
                /* 计算垂直速度 */
                float dt = (now - lastUpdateTime) / 1000.0f;
                if (dt > 0.001f && dt < 1.0f) {
                    float newSpeed = (data.altitude - lastAltitude) / dt;
                    /* 简单低通滤波 */
                    data.verticalSpeed = data.verticalSpeed * 0.8f + newSpeed * 0.2f;
                }
                lastAltitude = data.altitude;
                lastUpdateTime = now;
                
                data.valid = true;
                state = STATE_IDLE;
                return true;  // 新数据可用
            }
            return false;
        
        default:
            state = STATE_IDLE;
            return false;
    }
}

MS5611_Data_t* MS5611_GetData(void)
{
    return &data;
}

void MS5611_ResetAltitude(void)
{
    groundPressure = data.pressure;
    data.altitude = 0.0f;
    lastAltitude = 0.0f;
    data.verticalSpeed = 0.0f;
}

float MS5611_GetGroundPressure(void)
{
    return groundPressure;
}

/* ==================== 私有函数实现 ==================== */

static uint32_t ReadADC(uint8_t addr)
{
    uint8_t buf[3] = {0, 0, 0};
    
    /* MS5611 ADC 读取: 发送 0x00，STOP，然后读 3 字节 */
    /* 使用 WriteCmd(Stop) + ReadData(Start) 以严格遵守协议 */
    if (!BSP_I2C1_WriteCmd(addr, CMD_ADC_READ)) {
        return 0;
    }
    
    if (!BSP_I2C1_ReadData(addr, buf, 3)) {
        return 0;
    }
    
    uint32_t adc = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    
    /* 检查是否读到有效数据 (不应该是 0 或 0xFFFFFF) */
    if (adc == 0 || adc == 0xFFFFFF) {
        return 0;  // 无效数据
    }
    
    return adc;
}

static void Calculate(void)
{
    /* 温度计算 (第一阶补偿) */
    int32_t dT = D2 - ((uint32_t)C[5] << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * C[6]) >> 23);
    
    /* 气压计算 (第一阶补偿) */
    int64_t OFF  = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);
    
    /* 第二阶温度补偿 (低温修正) */
    int32_t T2 = 0;
    int64_t OFF2 = 0, SENS2 = 0;
    
    if (TEMP < 2000) {  // < 20°C
        T2 = ((int64_t)dT * dT) >> 31;
        int32_t t = TEMP - 2000;
        OFF2 = (5 * (int64_t)t * t) >> 1;
        SENS2 = (5 * (int64_t)t * t) >> 2;
        
        if (TEMP < -1500) {  // < -15°C
            int32_t t2 = TEMP + 1500;
            OFF2 += 7 * (int64_t)t2 * t2;
            SENS2 += (11 * (int64_t)t2 * t2) >> 1;
        }
    }
    
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    
    /* 最终气压 (0.01 mbar = 1 Pa) */
    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;
    
    /* 保存结果 */
    data.temperature = TEMP / 100.0f;  // °C
    data.pressure = (float)P;           // Pa
    
    /* 计算高度 */
    data.altitudeRaw = PressureToAltitude(data.pressure);
    data.altitude = PressureToAltitude(groundPressure) - data.altitudeRaw;
    // 注：相对高度 = 基准高度 - 当前绝对高度 (气压下降=高度上升)
    data.altitude = data.altitudeRaw - PressureToAltitude(groundPressure);
}

/**
 * @brief 国际气压高度公式
 * @param pressure 气压 (Pa)
 * @return 高度 (m)
 */
static float PressureToAltitude(float pressure)
{
    /* 标准大气模型: h = 44330 * (1 - (P/P0)^0.1903) */
    const float P0 = 101325.0f;  // 海平面标准气压
    return 44330.0f * (1.0f - powf(pressure / P0, 0.1903f));
}
