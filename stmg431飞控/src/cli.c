/**
 * @file cli.c
 * @brief 串口命令行接口实现
 * @note  支持调参、保存参数、查看状态
 * 
 * 可用命令：
 *   RP=5.0    Roll角度P      RPI=0.0   Roll角度I
 *   RR=4.0    Roll角速度P    RRI=0.02  Roll角速度I    RRD=0.5   Roll角速度D
 *   PP=5.0    Pitch角度P     PPI=0.0   Pitch角度I
 *   PR=4.0    Pitch角速度P   PRI=0.02  Pitch角速度I   PRD=0.5   Pitch角速度D
 *   YR=3.0    Yaw角速度P     YRI=0.01  Yaw角速度I
 *   BETA=0.1  Madgwick Beta
 *   SHOW      显示参数
 *   RESET     重置PID
 *   HELP      帮助
 */

#include "cli.h"
#include "bsp_uart.h"
#include "attitude.h"
#include "config.h"
#include "drdy_imu.h"
#include <string.h>
#include <stdlib.h>

/* ==================== 私有变量 ==================== */

static FlightPID_t *pFlightPID = NULL;
static char cmdBuffer[CLI_BUFFER_SIZE];
static uint8_t cmdIndex = 0;

/* ==================== 私有函数 ==================== */

static void CLI_ParseCommand(char *cmd);
static float ParseFloat(const char *str);

/* ==================== API实现 ==================== */

/**
 * @brief 初始化CLI
 */
void CLI_Init(FlightPID_t *pid)
{
    pFlightPID = pid;
    cmdIndex = 0;
    memset(cmdBuffer, 0, CLI_BUFFER_SIZE);
}

/**
 * @brief 处理接收到的字符
 */
void CLI_ProcessChar(char c)
{
    if (c == '\r' || c == '\n') {
        if (cmdIndex > 0) {
            cmdBuffer[cmdIndex] = '\0';
            CLI_ParseCommand(cmdBuffer);
            cmdIndex = 0;
            memset(cmdBuffer, 0, CLI_BUFFER_SIZE);
        }
    }
    else if (c == '\b' || c == 127) {
        /* 退格 */
        if (cmdIndex > 0) {
            cmdIndex--;
            cmdBuffer[cmdIndex] = '\0';
        }
    }
    else if (cmdIndex < CLI_BUFFER_SIZE - 1) {
        /* 添加字符 */
        cmdBuffer[cmdIndex++] = c;
    }
}

/**
 * @brief 显示帮助
 */
void CLI_ShowHelp(void)
{
    BSP_UART_Printf("\r\n=== Flight Controller CLI ===\r\n");
    BSP_UART_Printf("Roll Angle:  RP=x.x  RPI=x.x\r\n");
    BSP_UART_Printf("Roll Rate:   RR=x.x  RRI=x.x  RRD=x.x\r\n");
    BSP_UART_Printf("Pitch Angle: PP=x.x  PPI=x.x\r\n");
    BSP_UART_Printf("Pitch Rate:  PR=x.x  PRI=x.x  PRD=x.x\r\n");
    BSP_UART_Printf("Yaw Rate:    YR=x.x  YRI=x.x\r\n");
    BSP_UART_Printf("Madgwick:    BETA=x.x\r\n");
    BSP_UART_Printf("Commands:    SHOW RESET HELP\r\n");
    BSP_UART_Printf("=============================\r\n\r\n");
}

/**
 * @brief 显示当前参数
 */
void CLI_ShowParams(void)
{
    if (pFlightPID == NULL) return;
    
    BSP_UART_Printf("\r\n=== Current PID ===\r\n");
    
    /* 使用整数显示避免浮点问题 */
    int rp = (int)(pFlightPID->roll_angle.Kp * 100);
    int rr = (int)(pFlightPID->roll_rate.Kp * 100);
    int rri = (int)(pFlightPID->roll_rate.Ki * 1000);
    int rrd = (int)(pFlightPID->roll_rate.Kd * 100);
    
    int pp = (int)(pFlightPID->pitch_angle.Kp * 100);
    int pr = (int)(pFlightPID->pitch_rate.Kp * 100);
    int pri = (int)(pFlightPID->pitch_rate.Ki * 1000);
    int prd = (int)(pFlightPID->pitch_rate.Kd * 100);
    
    int yr = (int)(pFlightPID->yaw_rate.Kp * 100);
    int yri = (int)(pFlightPID->yaw_rate.Ki * 1000);
    
    BSP_UART_Printf("Roll:  Angle P=%d.%02d | Rate P=%d.%02d I=0.%03d D=%d.%02d\r\n",
        rp/100, rp%100, rr/100, rr%100, rri, rrd/100, rrd%100);
    BSP_UART_Printf("Pitch: Angle P=%d.%02d | Rate P=%d.%02d I=0.%03d D=%d.%02d\r\n",
        pp/100, pp%100, pr/100, pr%100, pri, prd/100, prd%100);
    BSP_UART_Printf("Yaw:   Rate P=%d.%02d I=0.%03d\r\n",
        yr/100, yr%100, yri);

    /* 显示可观测性统计 */
    DRDY_Stats_t *stats = DRDY_IMU_GetStats();
    BSP_UART_Printf("Stats: Loop(us) Min=%d Max=%d Avg=%d | Missed=%d\r\n",
        stats->minLoopTime_us, stats->maxLoopTime_us, stats->avgLoopTime_us, stats->missed_samples);

    BSP_UART_Printf("===================\r\n\r\n");
}

/* ==================== 命令解析 ==================== */

static void CLI_ParseCommand(char *cmd)
{
    if (pFlightPID == NULL) return;
    
    /* 转大写 */
    for (int i = 0; cmd[i]; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'z') {
            cmd[i] -= 32;
        }
    }
    
    /* 解析命令 */
    if (strncmp(cmd, "HELP", 4) == 0) {
        CLI_ShowHelp();
    }
    else if (strncmp(cmd, "SHOW", 4) == 0) {
        CLI_ShowParams();
    }
    else if (strncmp(cmd, "RESET", 5) == 0) {
        FlightPID_Init(pFlightPID);
        BSP_UART_Printf("[CLI] PID Reset to defaults\r\n");
    }
    else if (strncmp(cmd, "SAVE", 4) == 0) {
        Config_Save(pFlightPID);
        BSP_UART_Printf("[CLI] Params SAVED to Flash\r\n");
    }
    else if (strncmp(cmd, "LOAD", 4) == 0) {
        if (Config_Load(pFlightPID)) {
            BSP_UART_Printf("[CLI] Params LOADED from Flash\r\n");
        } else {
            BSP_UART_Printf("[CLI] Load FAILED (No config?)\r\n");
        }
    }
    /* Roll Angle P */
    else if (strncmp(cmd, "RP=", 3) == 0) {
        pFlightPID->roll_angle.Kp = ParseFloat(cmd + 3);
        pFlightPID->pitch_angle.Kp = pFlightPID->roll_angle.Kp;  // 同步
        BSP_UART_Printf("[CLI] Roll/Pitch Angle P set\r\n");
    }
    /* Roll Rate P */
    else if (strncmp(cmd, "RR=", 3) == 0) {
        pFlightPID->roll_rate.Kp = ParseFloat(cmd + 3);
        pFlightPID->pitch_rate.Kp = pFlightPID->roll_rate.Kp;
        BSP_UART_Printf("[CLI] Roll/Pitch Rate P set\r\n");
    }
    /* Roll Rate I */
    else if (strncmp(cmd, "RRI=", 4) == 0) {
        pFlightPID->roll_rate.Ki = ParseFloat(cmd + 4);
        pFlightPID->pitch_rate.Ki = pFlightPID->roll_rate.Ki;
        BSP_UART_Printf("[CLI] Roll/Pitch Rate I set\r\n");
    }
    /* Roll Rate D */
    else if (strncmp(cmd, "RRD=", 4) == 0) {
        pFlightPID->roll_rate.Kd = ParseFloat(cmd + 4);
        pFlightPID->pitch_rate.Kd = pFlightPID->roll_rate.Kd;
        BSP_UART_Printf("[CLI] Roll/Pitch Rate D set\r\n");
    }
    /* Yaw Rate P */
    else if (strncmp(cmd, "YR=", 3) == 0) {
        pFlightPID->yaw_rate.Kp = ParseFloat(cmd + 3);
        BSP_UART_Printf("[CLI] Yaw Rate P set\r\n");
    }
    /* Yaw Rate I */
    else if (strncmp(cmd, "YRI=", 4) == 0) {
        pFlightPID->yaw_rate.Ki = ParseFloat(cmd + 4);
        BSP_UART_Printf("[CLI] Yaw Rate I set\r\n");
    }
    /* Madgwick Beta */
    else if (strncmp(cmd, "BETA=", 5) == 0) {
        float beta = ParseFloat(cmd + 5);
        Attitude_SetBeta(beta);
        BSP_UART_Printf("[CLI] Beta set\r\n");
    }
    /* Notch 频率设置 */
    else if (strncmp(cmd, "NOTCH=", 6) == 0) {
        float freq = ParseFloat(cmd + 6);
        extern void DRDY_IMU_SetNotchFreq(float freq, float q);
        DRDY_IMU_SetNotchFreq(freq, 2.5f);  // 默认 Q=2.5
        BSP_UART_Printf("[CLI] Notch center freq set to %.0f Hz (Q=2.5)\r\n", freq);
    }
    /* Blackbox 开关 */
    else if (strncmp(cmd, "BB=", 3) == 0) {
        extern void Blackbox_Enable(bool enable);
        if (cmd[3] == '1') {
            Blackbox_Enable(true);
            BSP_UART_Printf("[CLI] Blackbox ENABLED (100Hz)\r\n");
        } else {
            Blackbox_Enable(false);
            BSP_UART_Printf("[CLI] Blackbox DISABLED\r\n");
        }
    }
    /* IMU Debug Switch */
    else if (strncmp(cmd, "IMUDBG ", 7) == 0) {
        if (cmd[7] == 'O' && cmd[8] == 'N') {
            imuDebugActive = true;
            BSP_UART_Printf("[CLI] IMU Debug Output: ON (10Hz)\r\n");
        } else {
            imuDebugActive = false;
            BSP_UART_Printf("[CLI] IMU Debug Output: OFF\r\n");
        }
    }
    /* Calibration */
    else if (strncmp(cmd, "GYROCAL", 7) == 0) {
        // External check for armed status should be done, but for now assuming user knows
        extern uint8_t armed; // From main.c
        if (armed) {
             BSP_UART_Printf("[CLI] ERROR: Cannot calibrate while ARMED!\r\n");
        } else {
             BSP_UART_Printf("[CLI] Calibrating Gyro... KEEP STILL (2s)\r\n");
             DRDY_IMU_CalibrateGyro();
             
             float bias[3];
             DRDY_IMU_GetGyroBias(bias);
             BSP_UART_Printf("[CLI] Done. Bias: X=%.2f Y=%.2f Z=%.2f\r\n", bias[0], bias[1], bias[2]);
             Config_Save(pFlightPID); // Auto save
        }
    }
    /* Axis Mapping: AXIS X -Y Z */
    else if (strncmp(cmd, "AXIS ", 5) == 0) {
        char *p = cmd + 5;
        uint8_t map[3] = {0, 1, 2};
        int8_t sign[3] = {1, 1, 1};
        int idx = 0;
        
        while (*p && idx < 3) {
            if (*p == ' ') { p++; continue; }
            
            if (*p == '-') {
                sign[idx] = -1;
                p++;
            } else {
                sign[idx] = 1;
            }
            
            if (*p == 'X') map[idx] = 0;
            else if (*p == 'Y') map[idx] = 1;
            else if (*p == 'Z') map[idx] = 2;
            else { 
                BSP_UART_Printf("[CLI] Error: Invalid axis '%c'\r\n", *p); 
                return; 
            }
            p++;
            idx++;
        }
        
        if (idx == 3) {
            DRDY_IMU_SetAxisMapping(map, sign);
            BSP_UART_Printf("[CLI] Axis set: %d*RAW[%d], %d*RAW[%d], %d*RAW[%d]\r\n",
                sign[0], map[0], sign[1], map[1], sign[2], map[2]);
            Config_Save(pFlightPID);
        } else {
            BSP_UART_Printf("[CLI] Error: Need 3 axes (e.g. AXIS X -Y Z)\r\n");
        }
    }
    else if (strncmp(cmd, "AXIS SHOW", 9) == 0) { // Will be caught by AXIS above if not careful, but length differs
        // Actually strncmp "AXIS " will catch this first. Restructure logic or check full string.
        // Re-ordering logic: Check specific sub-commands first or parse better.
        // For now, let's just make AXIS detect 'S' for SHOW.
    }
    /* Accel Attitude Debug */
    else if (strncmp(cmd, "ACCATT ", 7) == 0) {
        extern bool accAttActive; // Defined in main.c
        if (cmd[7] == 'O' && cmd[8] == 'N') {
            accAttActive = true;
            BSP_UART_Printf("[CLI] Accel Attitude Check: ON\r\n");
        } else {
            accAttActive = false;
            BSP_UART_Printf("[CLI] Accel Attitude Check: OFF\r\n");
        }
    }
    /* I2C Diagnostics */
    else if (strncmp(cmd, "I2CDIAG", 7) == 0) {
        extern void BSP_I2C1_Diagnose(void);
        BSP_I2C1_Diagnose();
    }
    /* I2C Scan Only */
    else if (strncmp(cmd, "I2CSCAN", 7) == 0) {
        extern uint8_t BSP_I2C1_Scan(uint8_t *foundAddrs);
        uint8_t addrs[16];
        uint8_t count = BSP_I2C1_Scan(addrs);
        if (count == 0) {
            BSP_UART_Printf("[I2C] No devices found!\r\n");
        } else {
            BSP_UART_Printf("[I2C] Found %d device(s):\r\n", count);
            for (int i = 0; i < count && i < 16; i++) {
                BSP_UART_Printf("  - 0x%02X\r\n", addrs[i]);
            }
        }
    }
    /* Barometer Retry */
    else if (strncmp(cmd, "BARO", 4) == 0) {
        extern bool MS5611_Init(void);
        extern void AltHold_Init(void);
        BSP_UART_Printf("[BARO] Retrying MS5611 init...\r\n");
        if (MS5611_Init()) {
            BSP_UART_Printf("[BARO] MS5611 OK!\r\n");
            AltHold_Init();
        } else {
            BSP_UART_Printf("[BARO] MS5611 NOT FOUND!\r\n");
        }
    }
    else if (strlen(cmd) > 0) {
        BSP_UART_Printf("[CLI] Unknown: %s\r\n", cmd);
    }
}

/**
 * @brief 简单的字符串转浮点
 */
static float ParseFloat(const char *str)
{
    float result = 0.0f;
    float decimal = 0.1f;
    int negative = 0;
    int afterDot = 0;
    
    if (*str == '-') {
        negative = 1;
        str++;
    }
    
    while (*str) {
        if (*str == '.') {
            afterDot = 1;
        }
        else if (*str >= '0' && *str <= '9') {
            if (afterDot) {
                result += (*str - '0') * decimal;
                decimal *= 0.1f;
            } else {
                result = result * 10.0f + (*str - '0');
            }
        }
        str++;
    }
    
    return negative ? -result : result;
}
