/**
 * @file cli.h
 * @brief 串口命令行接口 - 调参/调试
 * @note  通过串口发送命令调整PID参数
 * 
 * 命令格式：
 *   P1=5.0     设置角度环Roll P
 *   P2=4.0     设置角速度环Roll P
 *   SAVE       保存参数到Flash
 *   LOAD       从Flash加载参数
 *   SHOW       显示当前参数
 *   HELP       显示帮助
 */

#ifndef __CLI_H
#define __CLI_H

#include <stdint.h>
#include "pid.h"

/* 命令缓冲区大小 */
#define CLI_BUFFER_SIZE     64

/* ==================== API函数 ==================== */

/**
 * @brief 初始化CLI
 */
void CLI_Init(FlightPID_t *pid);

/**
 * @brief 处理接收到的字符 (在主循环或串口中断中调用)
 * @param c 接收到的字符
 */
void CLI_ProcessChar(char c);

/**
 * @brief 显示帮助信息
 */
void CLI_ShowHelp(void);

/**
 * @brief 显示当前参数
 */
void CLI_ShowParams(void);

#endif /* __CLI_H */
