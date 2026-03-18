/**
 * @file config.c
 * @brief 参数存储实现 - Flash模拟EEPROM
 * @note  STM32G431 Page Size = 2KB
 *        使用最后一页 (Page 63) 存放参数
 */

#include "config.h"
#include "stm32g4xx_hal.h"
#include "attitude.h"
#include "drdy_imu.h"

/*Flash配置*/
#define FLASH_CONFIG_PAGE_ADDR  0x0801F800  // 128KB Flash的最后一页起始地址 (0x08000000 + 128*1024 - 2048)
#define CONFIG_VERSION          0xAA02      // 版本升级，强制重置参数

/* ==================== 私有函数 ==================== */


/* ==================== API实现 ==================== */

/**
 * @brief 恢复默认参数
 */
void Config_ResetDefaults(FlightPID_t *pid)
{
    FlightPID_Init(pid);
    Attitude_SetBeta(0.1f);
    
    /* IMU 默认配置 (适配当前硬件) */
    uint8_t defMap[3] = {0, 1, 2};  // X, Y, Z
    int8_t defSign[3] = {-1, -1, 1}; // -X, -Y, +Z
    float defBias[3] = {0, 0, 0};
    
    DRDY_IMU_SetAxisMapping(defMap, defSign);
    DRDY_IMU_SetGyroBias(defBias);
}

/**
 * @brief 从Flash加载配置
 */
bool Config_Load(FlightPID_t *pid)
{
    Config_t *flashCfg = (Config_t*)FLASH_CONFIG_PAGE_ADDR;
    
    /* 检查版本号 */
    if (flashCfg->version != CONFIG_VERSION) {
        /* 如果版本不匹配，自动重置为默认值 */
        Config_ResetDefaults(pid);
        return false; 
    }
    
    /* 加载参数 */
    *pid = flashCfg->pid;
    Attitude_SetBeta(flashCfg->madgwickBeta);
    
    /* 加载 IMU 配置 */
    DRDY_IMU_SetAxisMapping((uint8_t*)flashCfg->axisMap, (int8_t*)flashCfg->axisSign);
    DRDY_IMU_SetGyroBias((float*)flashCfg->gyroBias);
    
    return true;
}

/**
 * @brief 保存配置到Flash
 */
void Config_Save(FlightPID_t *pid)
{
    Config_t cfg;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    
    /* 准备数据 */
    cfg.version = CONFIG_VERSION;
    cfg.pid = *pid;
    cfg.madgwickBeta = 0.1f; 
    
    /* 保存 IMU 配置 */
    DRDY_IMU_GetAxisMapping(cfg.axisMap, cfg.axisSign);
    DRDY_IMU_GetGyroBias(cfg.gyroBias);
    
    /* 解锁Flash */
    HAL_FLASH_Unlock();
    
    /* 擦除最后一页 */
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Page        = 63; 
    EraseInitStruct.NbPages     = 1;
    
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }
    
    /* 写入数据 (双字写入 64-bit) */
    uint64_t *pData = (uint64_t*)&cfg;
    uint32_t len = sizeof(Config_t) / 8 + 1;
    uint32_t addr = FLASH_CONFIG_PAGE_ADDR;
    
    for (uint32_t i = 0; i < len; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, pData[i]) != HAL_OK) {
            break;
        }
        addr += 8;
    }
    
    /* 上锁Flash */
    HAL_FLASH_Lock();
}
