/**
 * @file soft_spi_test.c
 * @brief 软件SPI测试 - 用于调试硬件接线
 */

#include "stm32g4xx_hal.h"
#include "bsp_uart.h"

/* 软件SPI引脚定义 - 和硬件SPI相同 */
#define SOFT_SCK_PIN    GPIO_PIN_5
#define SOFT_SCK_PORT   GPIOA
#define SOFT_MOSI_PIN   GPIO_PIN_7
#define SOFT_MOSI_PORT  GPIOA
#define SOFT_MISO_PIN   GPIO_PIN_6
#define SOFT_MISO_PORT  GPIOA
#define SOFT_CS_PIN     GPIO_PIN_0
#define SOFT_CS_PORT    GPIOB

/* 引脚操作宏 */
#define SCK_HIGH()    HAL_GPIO_WritePin(SOFT_SCK_PORT, SOFT_SCK_PIN, GPIO_PIN_SET)
#define SCK_LOW()     HAL_GPIO_WritePin(SOFT_SCK_PORT, SOFT_SCK_PIN, GPIO_PIN_RESET)
#define MOSI_HIGH()   HAL_GPIO_WritePin(SOFT_MOSI_PORT, SOFT_MOSI_PIN, GPIO_PIN_SET)
#define MOSI_LOW()    HAL_GPIO_WritePin(SOFT_MOSI_PORT, SOFT_MOSI_PIN, GPIO_PIN_RESET)
#define CS_HIGH()     HAL_GPIO_WritePin(SOFT_CS_PORT, SOFT_CS_PIN, GPIO_PIN_SET)
#define CS_LOW()      HAL_GPIO_WritePin(SOFT_CS_PORT, SOFT_CS_PIN, GPIO_PIN_RESET)
#define READ_MISO()   HAL_GPIO_ReadPin(SOFT_MISO_PORT, SOFT_MISO_PIN)

static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 42;  // 约170MHz/4
    while(count--);
}

/**
 * @brief 初始化软件SPI引脚
 */
void SoftSPI_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio = {0};
    
    /* SCK和MOSI配置为输出 */
    gpio.Pin = SOFT_SCK_PIN | SOFT_MOSI_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    
    /* MISO配置为输入 */
    gpio.Pin = SOFT_MISO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;  // 上拉，没连接时读到1
    HAL_GPIO_Init(GPIOA, &gpio);
    
    /* CS配置为输出 */
    gpio.Pin = SOFT_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* 初始状态 */
    CS_HIGH();
    SCK_LOW();  // 先用Mode 0 (CPOL=0)
    MOSI_LOW();
}

/**
 * @brief 软件SPI Mode 0 - 标准时序 (CPOL=0, CPHA=0)
 */
uint8_t SoftSPI_TransferByte_Mode0(uint8_t txData)
{
    uint8_t rxData = 0;
    
    SCK_LOW();
    delay_us(1);
    
    for (int i = 7; i >= 0; i--)
    {
        /* 在SCK低电平时设置MOSI */
        if (txData & (1 << i)) {
            MOSI_HIGH();
        } else {
            MOSI_LOW();
        }
        
        delay_us(5);  // 建立时间
        
        /* SCK上升沿 */
        SCK_HIGH();
        delay_us(8);  // 更长的等待时间
        
        /* 采样MISO */
        if (READ_MISO()) {
            rxData |= (1 << i);
        }
        
        /* SCK下降沿 */
        SCK_LOW();
        delay_us(3);
    }
    
    MOSI_LOW();
    return rxData;
}

/**
 * @brief 软件SPI读写一个字节 Mode 3 (CPOL=1, CPHA=1)
 */
uint8_t SoftSPI_TransferByte_Mode3(uint8_t txData)
{
    uint8_t rxData = 0;
    
    SCK_HIGH();  // Mode 3空闲为高
    delay_us(1);
    
    for (int i = 7; i >= 0; i--)
    {
        /* SCK下降沿 - 设置MOSI */
        SCK_LOW();
        
        if (txData & (1 << i)) {
            MOSI_HIGH();
        } else {
            MOSI_LOW();
        }
        
        delay_us(2);
        
        /* SCK上升沿 - 采样MISO */
        SCK_HIGH();
        delay_us(1);
        
        if (READ_MISO()) {
            rxData |= (1 << i);
        }
        
        delay_us(1);
    }
    
    return rxData;
}

/* 当前使用的模式 */
static uint8_t currentMode = 0;

uint8_t SoftSPI_TransferByte(uint8_t txData)
{
    if (currentMode == 0) {
        return SoftSPI_TransferByte_Mode0(txData);
    } else {
        return SoftSPI_TransferByte_Mode3(txData);
    }
}

void SoftSPI_SetMode(uint8_t mode)
{
    currentMode = mode;
    if (mode == 0) {
        SCK_LOW();
    } else {
        SCK_HIGH();
    }
}

/**
 * @brief 软件SPI读ICM42688寄存器
 */
uint8_t SoftSPI_ReadReg(uint8_t reg)
{
    uint8_t value;
    
    CS_LOW();
    delay_us(5);  // CS建立时间
    
    SoftSPI_TransferByte(reg | 0x80);  // 读命令（最高位=1）
    delay_us(2);  // 命令到数据间隔
    value = SoftSPI_TransferByte(0xFF);
    
    delay_us(2);
    CS_HIGH();
    delay_us(5);  // CS恢复时间
    
    return value;
}

/**
 * @brief 完整的GPIO测试
 */
void SoftSPI_GPIOTest(void)
{
    BSP_UART_Printf("\r\n===== GPIO Pin Test =====\r\n");
    
    /* 测试MISO的默认状态 */
    CS_HIGH();
    HAL_Delay(1);
    uint8_t miso_high = READ_MISO();
    
    CS_LOW();
    HAL_Delay(1);
    uint8_t miso_low = READ_MISO();
    
    CS_HIGH();
    
    BSP_UART_Printf("MISO when CS=HIGH: %d\r\n", miso_high);
    BSP_UART_Printf("MISO when CS=LOW:  %d\r\n", miso_low);
    
    if (miso_high == miso_low) {
        BSP_UART_Printf("[WARN] MISO does not change with CS!\r\n");
    }
    
    /* 尝试Mode 0 */
    BSP_UART_Printf("\r\n--- Testing SPI Mode 0 ---\r\n");
    SoftSPI_SetMode(0);
    uint8_t whoami_m0 = SoftSPI_ReadReg(0x75);
    BSP_UART_Printf("  Mode 0: WHO_AM_I = 0x%02X\r\n", whoami_m0);
    
    /* 尝试Mode 3 */
    BSP_UART_Printf("--- Testing SPI Mode 3 ---\r\n");
    SoftSPI_SetMode(3);
    uint8_t whoami_m3 = SoftSPI_ReadReg(0x75);
    BSP_UART_Printf("  Mode 3: WHO_AM_I = 0x%02X\r\n", whoami_m3);
    
    /* 判断哪个模式正确 */
    uint8_t whoami = 0;
    if (whoami_m0 == 0x47) {
        BSP_UART_Printf("\r\n[OK] Mode 0 works! ICM42688P detected.\r\n");
        SoftSPI_SetMode(0);
        whoami = whoami_m0;
    } else if (whoami_m3 == 0x47) {
        BSP_UART_Printf("\r\n[OK] Mode 3 works! ICM42688P detected.\r\n");
        SoftSPI_SetMode(3);
        whoami = whoami_m3;
    } else {
        BSP_UART_Printf("\r\n[ERROR] Neither mode works!\r\n");
        BSP_UART_Printf("Got Mode0=0x%02X, Mode3=0x%02X (expect 0x47)\r\n", whoami_m0, whoami_m3);
    }
    
    /* 如果成功，读取更多寄存器 */
    if (whoami == 0x47) {
        uint8_t intfCfg = SoftSPI_ReadReg(0x4C);
        uint8_t pwrMgmt = SoftSPI_ReadReg(0x4E);
        uint8_t bankSel = SoftSPI_ReadReg(0x76);
        
        BSP_UART_Printf("  INTF_CONFIG0 (0x4C) = 0x%02X\r\n", intfCfg);
        BSP_UART_Printf("  PWR_MGMT0    (0x4E) = 0x%02X\r\n", pwrMgmt);
        BSP_UART_Printf("  REG_BANK_SEL (0x76) = 0x%02X\r\n", bankSel);
    }
    
    BSP_UART_Printf("\r\nExpected WHO_AM_I values:\r\n");
    BSP_UART_Printf("  0x47 = ICM42688P\r\n");
    BSP_UART_Printf("  0x6B = ICM42605\r\n");
    BSP_UART_Printf("  0x00 = No response (check SCK/MOSI)\r\n");
    BSP_UART_Printf("  0xFF = MISO floating\r\n");
    BSP_UART_Printf("===========================\r\n\r\n");
}
