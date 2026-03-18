/**
 * @file drdy_imu_p99.c
 * @brief [Optimal-A] P99 计算实现 (用于 ISR 性能统计)
 * @note 快速分区算法，O(n) 平均复杂度
 */

#include "drdy_imu.h"
#include <string.h>

extern uint16_t isr_us_buffer[2048];
extern DRDY_Stats_t stats;

/* 分区函数 (快速选择算法) */
static uint16_t quickselect(uint16_t *arr, int left, int right, int k)
{
    if (left == right) return arr[left];
    
    int pivot = arr[right];
    int i = left;
    
    for (int j = left; j < right; j++) {
        if (arr[j] < pivot) {
            uint16_t temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
            i++;
        }
    }
    
    uint16_t temp = arr[i];
    arr[i] = arr[right];
    arr[right] = temp;
    
    if (i == k) return arr[i];
    else if (i < k) return quickselect(arr, i + 1, right, k);
    else return quickselect(arr, left, i - 1, k);
}

/**
 * @brief 计算 p99 执行时间
 */
void DRDY_IMU_CalcP99(void)
{
    /* 创建工作副本 (避免破坏环形缓冲) */
    static uint16_t work_buffer[2048];
    memcpy(work_buffer, isr_us_buffer, sizeof(isr_us_buffer));
    
    /* 计算 p99 索引 (2048 * 0.99 = 2027) */
    int p99_idx = 2027;
    
    stats.p99ExecTime_us = quickselect(work_buffer, 0, 2047, p99_idx);
}
