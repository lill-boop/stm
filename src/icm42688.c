/**
 * @file icm42688.c
 * @brief ICM42688P 6轴IMU驱动实现
 * @note  使用软件SPI，因为硬件SPI有兼容性问题
 */

#include "icm42688.h"
#include "bsp_spi.h"


/* 当前配置的灵敏度系数 */
static float gyroSensitivity = 16.4f;   // 默认 ±2000dps
static float accelSensitivity = 2048.0f; // 默认 ±16g

/* 陀螺仪灵敏度表 (LSB/°/s) */
static const float gyroSensTable[] = {
    16.4f,    // ±2000 dps
    32.8f,    // ±1000 dps
    65.5f,    // ±500 dps
    131.0f,   // ±250 dps
    262.0f,   // ±125 dps
    524.3f,   // ±62.5 dps
    1048.6f,  // ±31.25 dps
    2097.2f   // ±15.625 dps
};

/* 加速度计灵敏度表 (LSB/g) */
static const float accelSensTable[] = {
    2048.0f,  // ±16g
    4096.0f,  // ±8g
    8192.0f,  // ±4g
    16384.0f  // ±2g
};

/* 校准偏移量 */
static float gyroOffset[3] = {0};
static float accelOffset[3] = {0};

/**
 * @brief 写单个寄存器 (使用软件SPI)
 */
void ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
    ICM42688_CS_LOW();
    BSP_SPI1_TransmitReceive(reg & 0x7F);  // 写：最高位为0
    BSP_SPI1_TransmitReceive(value);
    ICM42688_CS_HIGH();
}

/**
 * @brief 读单个寄存器 (使用硬件SPI)
 */
uint8_t ICM42688_ReadReg(uint8_t reg)
{
    uint8_t value;
    ICM42688_CS_LOW();
    BSP_SPI1_TransmitReceive(reg | 0x80);  // 读：最高位为1
    value = BSP_SPI1_TransmitReceive(0xFF);
    ICM42688_CS_HIGH();
    return value;
}

/**
 * @brief 读多个连续寄存器 (使用硬件SPI)
 */
void ICM42688_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    ICM42688_CS_LOW();
    BSP_SPI1_TransmitReceive(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = BSP_SPI1_TransmitReceive(0xFF);
    }
    ICM42688_CS_HIGH();
}

/**
 * @brief 读取WHO_AM_I寄存器
 * @return 应该返回 0x47
 */
uint8_t ICM42688_ReadWhoAmI(void)
{
    return ICM42688_ReadReg(ICM42688_WHO_AM_I);
}

/**
 * @brief 初始化ICM42688
 * @return true=成功, false=失败
 */
bool ICM42688_Init(void)
{
    /* 等待上电稳定 */
    HAL_Delay(100);
    
    /* 软复位 */
    ICM42688_WriteReg(ICM42688_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);
    
    /* 检查WHO_AM_I */
    uint8_t whoami = ICM42688_ReadWhoAmI();
    if (whoami != ICM42688_DEVICE_ID) {
        return false;
    }
    
    /* 选择Bank 0 */
    ICM42688_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    
    /* 配置接口：SPI 4线模式 */
    ICM42688_WriteReg(ICM42688_INTF_CONFIG0, 0x00);
    
    /* 配置驱动强度 (可选) */
    ICM42688_WriteReg(ICM42688_DRIVE_CONFIG, 0x05);
    
    return true;
}

/**
 * @brief 配置陀螺仪和加速度计
 * @param gyroFS   陀螺仪量程
 * @param accelFS  加速度计量程
 * @param odr      输出数据速率
 * @return true=成功
 */
bool ICM42688_Configure(ICM42688_GyroFS_t gyroFS, ICM42688_AccelFS_t accelFS, ICM42688_ODR_t odr)
{
    /* 保存灵敏度 */
    gyroSensitivity = gyroSensTable[gyroFS];
    accelSensitivity = accelSensTable[accelFS];
    
    /* 确保在Bank 0 */
    ICM42688_WriteReg(ICM42688_REG_BANK_SEL, 0x00);
    
    /* 配置陀螺仪：量程 + ODR */
    uint8_t gyroConfig = ((uint8_t)gyroFS << 5) | (uint8_t)odr;
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG0, gyroConfig);
    
    /* 配置加速度计：量程 + ODR */
    uint8_t accelConfig = ((uint8_t)accelFS << 5) | (uint8_t)odr;
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, accelConfig);
    
    /* 配置陀螺仪滤波器 */
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG1, 0x00);
    
    /* 配置加速度计滤波器 */
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG1, 0x00);
    
    /* 使能陀螺仪和加速度计：低噪声模式 */
    // Bit[3:2] = GYRO_MODE = 11 (Low Noise)
    // Bit[1:0] = ACCEL_MODE = 11 (Low Noise)
    ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0F);
    
    /* 等待传感器启动 */
    HAL_Delay(50);
    
    return true;
}

/**
 * @brief 读取原始加速度数据
 */
void ICM42688_ReadRawAccel(ICM42688_RawData_t *data)
{
    uint8_t buffer[6];
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X1, buffer, 6);
    
    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

/**
 * @brief 读取原始陀螺仪数据
 */
void ICM42688_ReadRawGyro(ICM42688_RawData_t *data)
{
    uint8_t buffer[6];
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_X1, buffer, 6);
    
    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

/**
 * @brief 一次性读取所有原始数据（加速度+陀螺仪）
 */
void ICM42688_ReadAllRaw(ICM42688_RawData_t *accel, ICM42688_RawData_t *gyro)
{
    uint8_t buffer[12];
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X1, buffer, 12);
    
    accel->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel->z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    gyro->x = (int16_t)((buffer[6] << 8) | buffer[7]);
    gyro->y = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro->z = (int16_t)((buffer[10] << 8) | buffer[11]);
}

/**
 * @brief 读取缩放后的加速度数据 (单位: g)
 */
void ICM42688_ReadScaledAccel(ICM42688_ScaledData_t *data)
{
    ICM42688_RawData_t raw;
    ICM42688_ReadRawAccel(&raw);
    
    data->x = (float)raw.x / accelSensitivity;
    data->y = (float)raw.y / accelSensitivity;
    data->z = (float)raw.z / accelSensitivity;
}

/**
 * @brief 读取缩放后的陀螺仪数据 (单位: °/s)
 */
void ICM42688_ReadScaledGyro(ICM42688_ScaledData_t *data)
{
    ICM42688_RawData_t raw;
    ICM42688_ReadRawGyro(&raw);
    
    data->x = (float)raw.x / gyroSensitivity;
    data->y = (float)raw.y / gyroSensitivity;
    data->z = (float)raw.z / gyroSensitivity;
}

/**
 * @brief 读取温度 (单位: °C)
 */
float ICM42688_ReadTemperature(void)
{
    uint8_t buffer[2];
    ICM42688_ReadRegs(ICM42688_TEMP_DATA1, buffer, 2);
    
    int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
    // 温度公式: T = (TEMP_DATA / 132.48) + 25
    return ((float)rawTemp / 132.48f) + 25.0f;
}

/**
 * @brief 读取所有数据（加速度+陀螺仪+温度）
 * @note  陀螺仪带 Notch + 低通滤波
 */
void ICM42688_ReadAll(ICM42688_Data_t *data)
{
    uint8_t buffer[14];
    ICM42688_ReadRegs(ICM42688_TEMP_DATA1, buffer, 14);
    
    /* ==================== 滤波器状态 ==================== */
    /* 低通滤波器 (二阶 ~100Hz cutoff) */
    static float lpfX[2] = {0}, lpfY[2] = {0}, lpfZ[2] = {0};
    
    /* Notch 滤波器状态 (估算频率 ~150Hz, Q=5) */
    /* 对于 2212 电机 + 9450 桨, 悬停转速约 5400RPM, 主震动频率 ≈ 90-150Hz */
    static float notchX_z1 = 0, notchX_z2 = 0;
    static float notchY_z1 = 0, notchY_z2 = 0;
    static float notchZ_z1 = 0, notchZ_z2 = 0;
    
    /* Notch 滤波器系数 (150Hz @ 1000Hz, Q=5) */
    /* 预计算: omega = 2*pi*150/1000 = 0.942, alpha = sin(omega)/(2*Q) */
    #define NOTCH_B0   0.969f
    #define NOTCH_B1  -1.618f
    #define NOTCH_B2   0.969f
    #define NOTCH_A1  -1.618f
    #define NOTCH_A2   0.939f
    
    #define LPF_COEFF  0.20f   // 低通平滑系数
    
    // 温度
    int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->temperature = ((float)rawTemp / 132.48f) + 25.0f;
    
    // 加速度 (应用校准)
    int16_t ax = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t ay = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t az = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->accel.x = -(((float)ax / accelSensitivity) - accelOffset[0]);
    data->accel.y = -(((float)ay / accelSensitivity) - accelOffset[1]);
    data->accel.z = (float)az / accelSensitivity;
    
    // 陀螺仪 (应用校准)
    int16_t gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    float rawGyroX = -(((float)gx / gyroSensitivity) - gyroOffset[0]);
    float rawGyroY = -(((float)gy / gyroSensitivity) - gyroOffset[1]);
    float rawGyroZ = ((float)gz / gyroSensitivity) - gyroOffset[2];
    
    /* ==================== Notch 滤波 (消除电机震动) ==================== */
    float notchOutX = NOTCH_B0 * rawGyroX + notchX_z1;
    notchX_z1 = NOTCH_B1 * rawGyroX - NOTCH_A1 * notchOutX + notchX_z2;
    notchX_z2 = NOTCH_B2 * rawGyroX - NOTCH_A2 * notchOutX;
    
    float notchOutY = NOTCH_B0 * rawGyroY + notchY_z1;
    notchY_z1 = NOTCH_B1 * rawGyroY - NOTCH_A1 * notchOutY + notchY_z2;
    notchY_z2 = NOTCH_B2 * rawGyroY - NOTCH_A2 * notchOutY;
    
    float notchOutZ = NOTCH_B0 * rawGyroZ + notchZ_z1;
    notchZ_z1 = NOTCH_B1 * rawGyroZ - NOTCH_A1 * notchOutZ + notchZ_z2;
    notchZ_z2 = NOTCH_B2 * rawGyroZ - NOTCH_A2 * notchOutZ;
    
    /* ==================== 二阶低通滤波 ==================== */
    lpfX[0] += LPF_COEFF * (notchOutX - lpfX[0]);
    lpfX[1] += LPF_COEFF * (lpfX[0] - lpfX[1]);
    
    lpfY[0] += LPF_COEFF * (notchOutY - lpfY[0]);
    lpfY[1] += LPF_COEFF * (lpfY[0] - lpfY[1]);
    
    lpfZ[0] += LPF_COEFF * (notchOutZ - lpfZ[0]);
    lpfZ[1] += LPF_COEFF * (lpfZ[0] - lpfZ[1]);
    
    data->gyro.x = lpfX[1];
    data->gyro.y = lpfY[1];
    data->gyro.z = lpfZ[1];
}

/* 校准偏移量 */


/**
 * @brief 检查数据是否就绪
 */
bool ICM42688_DataReady(void)
{
    uint8_t status = ICM42688_ReadReg(ICM42688_INT_STATUS);
    return (status & 0x08) != 0;  // DATA_RDY_INT
}

/**
 * @brief 简单的启动校准 (需保持静止水平)
 */
void ICM42688_Calibrate(void)
{
    float gSum[3] = {0};
    float aSum[3] = {0};
    int samples = 1000;
    ICM42688_RawData_t rawAccel, rawGyro;
    
    // 丢弃前100个样本
    for(int i=0; i<100; i++) {
        ICM42688_ReadAllRaw(&rawAccel, &rawGyro);
        HAL_Delay(1);
    }
    
    // 采样
    for(int i=0; i<samples; i++) {
        ICM42688_ReadAllRaw(&rawAccel, &rawGyro);
        
        gSum[0] += rawGyro.x;
        gSum[1] += rawGyro.y;
        gSum[2] += rawGyro.z;
        
        aSum[0] += rawAccel.x;
        aSum[1] += rawAccel.y;
        // Z轴是重力方向，不校准为0
        
        HAL_Delay(1);
    }
    
    // 计算平均值
    gyroOffset[0] = (gSum[0] / samples) / gyroSensitivity;
    gyroOffset[1] = (gSum[1] / samples) / gyroSensitivity;
    gyroOffset[2] = (gSum[2] / samples) / gyroSensitivity;
    
    // 加速度计偏移 (转换为g)
    accelOffset[0] = (aSum[0] / samples) / accelSensitivity;
    accelOffset[1] = (aSum[1] / samples) / accelSensitivity;
    // Z轴偏移暂不处理
    
    // 强制打印校准结果 (如果使用了printf重定向)
    // printf("Calib: G[%.2f %.2f %.2f] A[%.2f %.2f]\r\n", 
    //        gyroOffset[0], gyroOffset[1], gyroOffset[2],
    //        accelOffset[0], accelOffset[1]);
}
