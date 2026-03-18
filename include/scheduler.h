/**
 * @file scheduler.h
 * @brief 任务调度器 - 分频执行不同优先级任务
 * 
 * 调度策略:
 * - 1000Hz: IMU读取 → 滤波 → PID → 混控 → 输出 (最高优先级)
 * - 100Hz:  电池检测、状态机、蜂鸣器
 * - 10Hz:   CLI、串口打印、LED
 */

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

/* 任务频率定义 */
#define TASK_FREQ_1KHZ      1000
#define TASK_FREQ_100HZ     100
#define TASK_FREQ_10HZ      10

/* 任务分频器 (基于1kHz主循环) */
#define TASK_DIV_100HZ      10      // 1000/100 = 10
#define TASK_DIV_10HZ       100     // 1000/10 = 100

/* 任务ID */
typedef enum {
    TASK_FLIGHT_CONTROL,    // 1kHz - 飞行控制
    TASK_BATTERY,           // 100Hz - 电池检测
    TASK_BUZZER,            // 100Hz - 蜂鸣器
    TASK_STATE,             // 100Hz - 状态机
    TASK_CLI,               // 10Hz - CLI处理
    TASK_TELEMETRY,         // 10Hz - 遥测/日志
    TASK_LED,               // 10Hz - LED指示
    TASK_COUNT
} TaskID_t;

/* 调度器统计 */
typedef struct {
    uint32_t loopCount;             // 主循环计数
    uint32_t maxLoopTime_us;        // 最大循环时间 (微秒)
    uint32_t avgLoopTime_us;        // 平均循环时间
    uint32_t lastLoopTime_us;       // 上次循环时间
    uint32_t taskTime[TASK_COUNT];  // 各任务耗时
} SchedulerStats_t;

/* API */
void Scheduler_Init(void);
void Scheduler_Run(void);  // 在主循环调用

/* 检查是否该执行某频率的任务 */
bool Scheduler_ShouldRun100Hz(void);
bool Scheduler_ShouldRun10Hz(void);

/* DWT 计时器 (微秒级精度) */
void Scheduler_StartMeasure(void);
uint32_t Scheduler_EndMeasure(void);  // 返回耗时 (us)

/* 获取统计 */
SchedulerStats_t* Scheduler_GetStats(void);
void Scheduler_ResetStats(void);

#endif /* __SCHEDULER_H */
