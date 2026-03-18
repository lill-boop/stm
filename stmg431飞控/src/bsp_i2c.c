/**
 * @file bsp_i2c.c
 * @brief 硬件 I2C1 驱动实现 - STM32G431飞控项目
 * @note  使用 I2C1 (PA15=SCL, PB7=SDA), 100kHz 标准模式
 */

#include "bsp_i2c.h"
#include "stm32g4xx_hal.h"

/* ==================== 私有变量 ==================== */
static I2C_HandleTypeDef hi2c1;

/* ==================== API 实现 ==================== */

void BSP_I2C1_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ==================== 暴力复位 (I2C Bus Clear) ==================== */
    /* 在初始化前发送9个时钟脉冲，解救死锁的从设备 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 1. SCL (PA15) -> Output PP
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 2. SDA (PB7) -> Input (Check if high)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 3. 9 Clocks
    for(int i=0; i<9; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        for(volatile int d=0; d<200; d++);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        for(volatile int d=0; d<200; d++);
    }
    
    // 4. STOP Condition
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for(volatile int d=0; d<200; d++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    for(volatile int d=0; d<200; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    
    /* ==================== 正常初始化 ==================== */

    /* STM32G4 没有 JTAG，PA15 在配置为 AF 后自动释放 */
    
    /* 配置 PA15 = I2C1_SCL (AF4) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 使用内部上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;     // AF4 = I2C1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置 PB7 = I2C1_SDA (AF4) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 配置 I2C1 - 100kHz 标准模式 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x30A0A7FB;  // 100kHz @ 170MHz
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        return;
    }
    
    /* 配置滤波器 */
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

bool BSP_I2C1_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, 
                                data, len, I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool BSP_I2C1_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT,
                               data, len, I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool BSP_I2C1_WriteCmd(uint8_t devAddr, uint8_t cmd)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &cmd, 1, I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool BSP_I2C1_ReadData(uint8_t devAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, data, len, I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

/* ==================== I2C 诊断功能 ==================== */

/**
 * @brief 检查 I2C 总线状态 (SDA/SCL 电平)
 * @return 0=正常(都为高), 1=SCL被拉低, 2=SDA被拉低, 3=都被拉低
 */
uint8_t BSP_I2C1_CheckBusState(void)
{
    /* 暂时将引脚配置为 GPIO 输入读取电平 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* PA15 (SCL) 配置为浮空输入 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* PB7 (SDA) 配置为浮空输入 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_Delay(1);  // 等待电平稳定
    
    uint8_t scl = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    uint8_t sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    
    /* 恢复 I2C 功能 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    uint8_t result = 0;
    if (!scl) result |= 1;  // SCL 被拉低
    if (!sda) result |= 2;  // SDA 被拉低
    
    return result;
}

/**
 * @brief 发送 9 个时钟脉冲尝试解除 I2C 总线死锁
 * @note  当从设备卡在等待 ACK 的状态时，主机可以通过发送时钟脉冲让从设备释放 SDA
 * @return true=恢复成功 (SDA 变高), false=恢复失败
 */
bool BSP_I2C1_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 先禁用 I2C1 外设 */
    HAL_I2C_DeInit(&hi2c1);
    
    /* PA15 (SCL) 配置为推挽输出 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出用于产生时钟
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* PB7 (SDA) 配置为开漏输入 (用于读取状态) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // 内部上拉
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 发送 9 个时钟脉冲 */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // SCL = LOW
        for (volatile int d = 0; d < 100; d++);  // ~10μs @ 170MHz
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);    // SCL = HIGH
        for (volatile int d = 0; d < 100; d++);
        
        /* 检查 SDA 是否释放 */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
            break;  // SDA 已高，可以提前退出
        }
    }
    
    /* 发送 STOP 条件: SDA 从 LOW 到 HIGH (当 SCL=HIGH) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  // SDA = LOW
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // SCL = HIGH
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);    // SDA = HIGH (STOP)
    for (volatile int d = 0; d < 100; d++);
    
    /* 读取 SDA 最终状态 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    bool recovered = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
    
    /* 重新初始化 I2C */
    BSP_I2C1_Init();
    
    return recovered;
}

/**
 * @brief 扫描 I2C 总线上的设备
 * @param foundAddrs 找到的设备地址数组 (最多 16 个)
 * @return 找到的设备数量
 */
uint8_t BSP_I2C1_Scan(uint8_t *foundAddrs)
{
    uint8_t count = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        /* 使用 HAL_I2C_IsDeviceReady 探测设备 */
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
            if (foundAddrs && count < 16) {
                foundAddrs[count] = addr;
            }
            count++;
        }
    }
    
    return count;
}

/**
 * @brief 完整的 I2C 总线诊断
 * @note  会通过 BSP_UART_Printf 输出诊断信息
 */
void BSP_I2C1_Diagnose(void)
{
    extern void BSP_UART_Printf(const char *fmt, ...);
    
    BSP_UART_Printf("\r\n======== I2C1 Bus Diagnosis ========\r\n");
    
    /* 0. 打印 I2C 外设寄存器状态 */
    uint32_t isr = I2C1->ISR;
    BSP_UART_Printf("[Step 0] I2C1 ISR=0x%08lX\r\n", isr);
    BSP_UART_Printf("         BUSY=%ld BERR=%ld ARLO=%ld NACKF=%ld STOPF=%ld\r\n",
        (isr & I2C_ISR_BUSY)  ? 1 : 0,
        (isr & I2C_ISR_BERR)  ? 1 : 0,
        (isr & I2C_ISR_ARLO)  ? 1 : 0,
        (isr & I2C_ISR_NACKF) ? 1 : 0,
        (isr & I2C_ISR_STOPF) ? 1 : 0);
    
    if (isr & I2C_ISR_BUSY) {
        BSP_UART_Printf("         -> I2C is BUSY! May need bus recovery.\r\n");
    }
    if (isr & I2C_ISR_BERR) {
        BSP_UART_Printf("         -> Bus ERROR detected (misplaced START/STOP)\r\n");
    }
    if (isr & I2C_ISR_ARLO) {
        BSP_UART_Printf("         -> Arbitration LOST (multi-master conflict?)\r\n");
    }
    
    /* 1. 检查总线状态 */
    uint8_t busState = BSP_I2C1_CheckBusState();
    BSP_UART_Printf("[Step 1] Bus State: ");
    switch (busState) {
        case 0: BSP_UART_Printf("OK (SCL=HIGH, SDA=HIGH)\r\n"); break;
        case 1: BSP_UART_Printf("ERROR! SCL stuck LOW!\r\n"); break;
        case 2: BSP_UART_Printf("WARNING: SDA stuck LOW (possible bus lock)\r\n"); break;
        case 3: BSP_UART_Printf("ERROR! Both SCL and SDA stuck LOW!\r\n"); break;
    }
    
    /* 2. 如果 SDA 被拉低，尝试恢复 */
    if (busState & 2) {
        BSP_UART_Printf("[Step 2] Attempting bus recovery (9 clock pulses)...\r\n");
        if (BSP_I2C1_BusRecovery()) {
            BSP_UART_Printf("         Recovery SUCCESS! SDA is now HIGH.\r\n");
        } else {
            BSP_UART_Printf("         Recovery FAILED! SDA still LOW.\r\n");
            BSP_UART_Printf("         -> Hardware issue likely (check wiring!)\r\n");
            return;
        }
    }
    
    /* 3. 扫描设备 */
    BSP_UART_Printf("[Step 3] Scanning I2C bus (0x08 - 0x77)...\r\n");
    uint8_t addrs[16];
    uint8_t count = BSP_I2C1_Scan(addrs);
    
    if (count == 0) {
        BSP_UART_Printf("         No devices found!\r\n");
        BSP_UART_Printf("\r\n[Possible Causes]\r\n");
        BSP_UART_Printf("  1. Breadboard contact issue (most likely!)\r\n");
        BSP_UART_Printf("  2. Weak pull-ups (add 4.7k external resistors)\r\n");
        BSP_UART_Printf("  3. Wrong wiring (check VCC, GND, SCL, SDA)\r\n");
        BSP_UART_Printf("  4. Device damaged\r\n");
    } else {
        BSP_UART_Printf("         Found %d device(s):\r\n", count);
        for (int i = 0; i < count && i < 16; i++) {
            BSP_UART_Printf("           - 0x%02X", addrs[i]);
            if (addrs[i] == 0x77) BSP_UART_Printf(" (MS5611 CSB=GND)");
            else if (addrs[i] == 0x76) BSP_UART_Printf(" (MS5611 CSB=VCC or BMP280)");
            BSP_UART_Printf("\r\n");
        }
    }
    
    BSP_UART_Printf("====================================\r\n\r\n");
}
