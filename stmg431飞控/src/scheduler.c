/**
 * @file scheduler.c
 * @brief 任务调度器实现
 * @note  使用 DWT Cycle Counter 实现微秒级计时
 */

#include "scheduler.h"
#include "stm32g4xx_hal.h"

/* ==================== DWT 周期计数器 ==================== */
/* 
 * DWT (Data Watchpoint and Trace) 提供 CPU 周期级计时
 * 170MHz -> 1 cycle = 5.88ns
 */

#define DWT_CONTROL  (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT   (*(volatile uint32_t*)0xE0001004)
#define DWT_LAR      (*(volatile uint32_t*)0xE0001FB0)
#define SCB_DEMCR    (*(volatile uint32_t*)0xE000EDFC)

/* CPU 频率 */
#define CPU_FREQ_MHZ    170

/* ==================== 私有变量 ==================== */

static SchedulerStats_t stats = {0};
static uint32_t measureStart = 0;
static uint32_t loop100HzCounter = 0;
static uint32_t loop10HzCounter = 0;

/* ==================== API 实现 ==================== */

/**
 * @brief 初始化调度器
 */
void Scheduler_Init(void)
{
    /* 使能 DWT */
    SCB_DEMCR |= 0x01000000;   // 使能 Trace
    DWT_LAR = 0xC5ACCE55;      // 解锁 DWT
    DWT_CYCCNT = 0;            // 清零计数器
    DWT_CONTROL |= 1;          // 使能周期计数
    
    /* 清零统计 */
    Scheduler_ResetStats();
}

/**
 * @brief 主调度器运行 (在1kHz循环中调用)
 */
void Scheduler_Run(void)
{
    stats.loopCount++;
    loop100HzCounter++;
    loop10HzCounter++;
}

/**
 * @brief 检查是否执行 100Hz 任务
 */
bool Scheduler_ShouldRun100Hz(void)
{
    if (loop100HzCounter >= TASK_DIV_100HZ) {
        loop100HzCounter = 0;
        return true;
    }
    return false;
}

/**
 * @brief 检查是否执行 10Hz 任务
 */
bool Scheduler_ShouldRun10Hz(void)
{
    if (loop10HzCounter >= TASK_DIV_10HZ) {
        loop10HzCounter = 0;
        return true;
    }
    return false;
}

/**
 * @brief 开始计时
 */
void Scheduler_StartMeasure(void)
{
    measureStart = DWT_CYCCNT;
}

/**
 * @brief 结束计时并返回耗时 (微秒)
 */
uint32_t Scheduler_EndMeasure(void)
{
    uint32_t cycles = DWT_CYCCNT - measureStart;
    return cycles / CPU_FREQ_MHZ;  // cycles -> us
}

/**
 * @brief 获取统计数据
 */
SchedulerStats_t* Scheduler_GetStats(void)
{
    return &stats;
}

/**
 * @brief 重置统计
 */
void Scheduler_ResetStats(void)
{
    stats.loopCount = 0;
    stats.maxLoopTime_us = 0;
    stats.avgLoopTime_us = 0;
    stats.lastLoopTime_us = 0;
    for (int i = 0; i < TASK_COUNT; i++) {
        stats.taskTime[i] = 0;
    }
}
