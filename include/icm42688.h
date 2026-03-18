/**
 * @file icm42688.h
 * @brief ICM42688P 6轴IMU驱动头文件
 * @note  支持SPI通信，最高8MHz
 */

#ifndef __ICM42688_H
#define __ICM42688_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

/* ==================== ICM42688 寄存器地址 ==================== */
// User Bank 0
#define ICM42688_DEVICE_CONFIG     0x11
#define ICM42688_DRIVE_CONFIG      0x13
#define ICM42688_INT_CONFIG        0x14
#define ICM42688_FIFO_CONFIG       0x16
#define ICM42688_TEMP_DATA1        0x1D
#define ICM42688_TEMP_DATA0        0x1E
#define ICM42688_ACCEL_DATA_X1     0x1F
#define ICM42688_ACCEL_DATA_X0     0x20
#define ICM42688_ACCEL_DATA_Y1     0x21
#define ICM42688_ACCEL_DATA_Y0     0x22
#define ICM42688_ACCEL_DATA_Z1     0x23
#define ICM42688_ACCEL_DATA_Z0     0x24
#define ICM42688_GYRO_DATA_X1      0x25
#define ICM42688_GYRO_DATA_X0      0x26
#define ICM42688_GYRO_DATA_Y1      0x27
#define ICM42688_GYRO_DATA_Y0      0x28
#define ICM42688_GYRO_DATA_Z1      0x29
#define ICM42688_GYRO_DATA_Z0      0x2A
#define ICM42688_TMST_FSYNCH       0x2B
#define ICM42688_TMST_FSYNCL       0x2C
#define ICM42688_INT_STATUS        0x2D
#define ICM42688_FIFO_COUNTH       0x2E
#define ICM42688_FIFO_COUNTL       0x2F
#define ICM42688_FIFO_DATA         0x30
#define ICM42688_APEX_DATA0        0x31
#define ICM42688_APEX_DATA1        0x32
#define ICM42688_APEX_DATA2        0x33
#define ICM42688_APEX_DATA3        0x34
#define ICM42688_APEX_DATA4        0x35
#define ICM42688_APEX_DATA5        0x36
#define ICM42688_INT_STATUS2       0x37
#define ICM42688_INT_STATUS3       0x38
#define ICM42688_SIGNAL_PATH_RESET 0x4B
#define ICM42688_INTF_CONFIG0      0x4C
#define ICM42688_INTF_CONFIG1      0x4D
#define ICM42688_PWR_MGMT0         0x4E
#define ICM42688_GYRO_CONFIG0      0x4F
#define ICM42688_ACCEL_CONFIG0     0x50
#define ICM42688_GYRO_CONFIG1      0x51
#define ICM42688_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688_ACCEL_CONFIG1     0x53
#define ICM42688_TMST_CONFIG       0x54
#define ICM42688_APEX_CONFIG0      0x56
#define ICM42688_SMD_CONFIG        0x57
#define ICM42688_FIFO_CONFIG1      0x5F
#define ICM42688_FIFO_CONFIG2      0x60
#define ICM42688_FIFO_CONFIG3      0x61
#define ICM42688_FSYNC_CONFIG      0x62
#define ICM42688_INT_CONFIG0       0x63
#define ICM42688_INT_CONFIG1       0x64
#define ICM42688_INT_SOURCE0       0x65
#define ICM42688_INT_SOURCE1       0x66
#define ICM42688_INT_SOURCE3       0x68
#define ICM42688_INT_SOURCE4       0x69
#define ICM42688_FIFO_LOST_PKT0    0x6C
#define ICM42688_FIFO_LOST_PKT1    0x6D
#define ICM42688_SELF_TEST_CONFIG  0x70
#define ICM42688_WHO_AM_I          0x75
#define ICM42688_REG_BANK_SEL      0x76

/* WHO_AM_I 返回值 */
#define ICM42688_DEVICE_ID         0x47

/* ==================== 配置枚举 ==================== */

/* 陀螺仪量程 */
typedef enum {
    GYRO_FS_2000DPS = 0,   // ±2000 °/s
    GYRO_FS_1000DPS = 1,   // ±1000 °/s
    GYRO_FS_500DPS  = 2,   // ±500 °/s
    GYRO_FS_250DPS  = 3,   // ±250 °/s
    GYRO_FS_125DPS  = 4,   // ±125 °/s
    GYRO_FS_62_5DPS = 5,   // ±62.5 °/s
    GYRO_FS_31_25DPS = 6,  // ±31.25 °/s
    GYRO_FS_15_625DPS = 7  // ±15.625 °/s
} ICM42688_GyroFS_t;

/* 加速度计量程 */
typedef enum {
    ACCEL_FS_16G = 0,      // ±16g
    ACCEL_FS_8G  = 1,      // ±8g
    ACCEL_FS_4G  = 2,      // ±4g
    ACCEL_FS_2G  = 3       // ±2g
} ICM42688_AccelFS_t;

/* 输出数据速率 ODR */
typedef enum {
    ODR_32KHZ   = 1,
    ODR_16KHZ   = 2,
    ODR_8KHZ    = 3,
    ODR_4KHZ    = 4,
    ODR_2KHZ    = 5,
    ODR_1KHZ    = 6,
    ODR_200HZ   = 7,
    ODR_100HZ   = 8,
    ODR_50HZ    = 9,
    ODR_25HZ    = 10,
    ODR_12_5HZ  = 11,
    ODR_6_25HZ  = 12,      // 仅加速度计
    ODR_3_125HZ = 13,      // 仅加速度计
    ODR_1_5625HZ = 14,     // 仅加速度计
    ODR_500HZ   = 15
} ICM42688_ODR_t;

/* ==================== 数据结构 ==================== */

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ICM42688_RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} ICM42688_ScaledData_t;

typedef struct {
    ICM42688_ScaledData_t gyro;     // 陀螺仪 (°/s)
    ICM42688_ScaledData_t accel;    // 加速度计 (g)
    float temperature;               // 温度 (°C)
} ICM42688_Data_t;

/* ==================== 函数声明 ==================== */

bool ICM42688_Init(void);
uint8_t ICM42688_ReadWhoAmI(void);
bool ICM42688_Configure(ICM42688_GyroFS_t gyroFS, ICM42688_AccelFS_t accelFS, ICM42688_ODR_t odr);

void ICM42688_ReadRawAccel(ICM42688_RawData_t *data);
void ICM42688_ReadRawGyro(ICM42688_RawData_t *data);
void ICM42688_ReadAllRaw(ICM42688_RawData_t *accel, ICM42688_RawData_t *gyro);

void ICM42688_ReadScaledAccel(ICM42688_ScaledData_t *data);
void ICM42688_ReadScaledGyro(ICM42688_ScaledData_t *data);
void ICM42688_ReadAll(ICM42688_Data_t *data);

float ICM42688_ReadTemperature(void);
bool ICM42688_DataReady(void);
void ICM42688_Calibrate(void);

/* 底层读写 */
void ICM42688_WriteReg(uint8_t reg, uint8_t value);
uint8_t ICM42688_ReadReg(uint8_t reg);
void ICM42688_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len);

#endif /* __ICM42688_H */
