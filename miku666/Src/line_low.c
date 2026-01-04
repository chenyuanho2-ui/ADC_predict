#include "line_low.h"
#include <stdio.h>
#include <string.h> // for memset
#include "Calc_Median.h"         // 复用你之前的代码
#include "Calc_Sliding_Average.h"// 复用你之前的代码

// --- 参数配置 ---
#define SAMPLE_INTERVAL_MS  50   // 采样间隔约50ms
#define TIME_MEDIAN_SEC     0.5f // 中值滤波窗口时间
#define TIME_SLIDING_SEC    1.0f // 滑动滤波窗口时间

// 计算需要的缓冲区大小
// 0.5s / 0.05s = 10个点
#define LEN_MED_BUF  (int)(TIME_MEDIAN_SEC * 1000 / SAMPLE_INTERVAL_MS)
// 1.0s / 0.05s = 20个点
#define LEN_AVG_BUF  (int)(TIME_SLIDING_SEC * 1000 / SAMPLE_INTERVAL_MS)

#define RESULT_COUNT_TARGET 5    // 需要收集的结果数量

// --- 内部变量 ---
static LineLow_State_t state = LINE_LOW_IDLE;

// 第一级：中值滤波缓冲
static float buf_raw[LEN_MED_BUF];
static uint16_t count_raw = 0;

// 第二级：滑动滤波缓冲 (存放的是计算出来的中值)
static float buf_stage2[LEN_AVG_BUF];
static uint16_t count_stage2 = 0;

// 结果收集
static float final_results[RESULT_COUNT_TARGET];
static uint8_t result_idx = 0;

// --- 函数实现 ---

// 重置所有缓冲区和计数器
void LineLow_Start(void) {
    count_raw = 0;
    count_stage2 = 0;
    result_idx = 0;
    memset(buf_raw, 0, sizeof(buf_raw));
    memset(buf_stage2, 0, sizeof(buf_stage2));
    
    state = LINE_LOW_BUSY;
    printf("[LineLow] Test Start! Wait 1.5s...\r\n");
}

LineLow_State_t LineLow_GetState(void) {
    return state;
}

// 简单的移位插入函数（模拟队列）
// 把 new_val 放入 buffer，如果满了就挤掉最早的
static void Push_Buffer(float* buffer, uint16_t* count, int max_len, float new_val) {
    if (*count < max_len) {
        buffer[*count] = new_val;
        (*count)++;
    } else {
        // 移位
        for (int i = 0; i < max_len - 1; i++) {
            buffer[i] = buffer[i+1];
        }
        buffer[max_len - 1] = new_val;
    }
}

// 核心处理流程
uint8_t LineLow_Process(uint32_t raw_adc) {
    if (state != LINE_LOW_BUSY) return 0;

    // 1. 【第一级】输入原始数据到 Raw Buffer
    Push_Buffer(buf_raw, &count_raw, LEN_MED_BUF, (float)raw_adc);

    // 2. 检查第一级是否填满 (0.5s 数据是否凑齐)
    // 注意：只有填满后，产生的中值才具备 0.5s 的参考意义
    if (count_raw < LEN_MED_BUF) {
        // 数据还不够，无法进行下一级计算
        return 0;
    }

    // 3. 计算当前窗口的中值
    // 调用你之前写的 Calc_Median.c 里的函数
    float current_median = Calc_Median(buf_raw, count_raw);

    // 4. 【第二级】把中值结果输入到 Stage2 Buffer
    Push_Buffer(buf_stage2, &count_stage2, LEN_AVG_BUF, current_median);

    // 5. 检查第二级是否填满 (1.0s 数据是否凑齐)
    if (count_stage2 < LEN_AVG_BUF) {
        // 数据还不够
        return 0;
    }

    // 6. 计算最终的滑动平均值
    // 到这里时，系统已经运行了 0.5s + 1.0s = 1.5s
    float final_val = Calc_Sliding_Average(buf_stage2, count_stage2);

    // 7. 收集结果 (我们需要前 5 个产生的值)
    if (result_idx < RESULT_COUNT_TARGET) {
        final_results[result_idx] = final_val;
        result_idx++;
        
        // 打印调试信息(可选)
        // printf("Got Result[%d]: %.2f\r\n", result_idx, final_val);
    }

    // 8. 收集满 5 个后，寻找最大值并打印
    if (result_idx >= RESULT_COUNT_TARGET) {
        float max_val = 0.0f;
        for (int i = 0; i < RESULT_COUNT_TARGET; i++) {
            if (final_results[i] > max_val) {
                max_val = final_results[i];
            }
        }
        
        printf("[LineLow] Result: Max of 5 is %.2f\r\n", max_val);
        
        state = LINE_LOW_IDLE; // 任务结束
        return 1;
    }

    return 0;
}
