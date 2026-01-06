#include "line_cusum.h"
#include "cusum.h"
#include "Calc_Median.h"
#include "Calc_Sliding_Average.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// --- 参数配置 ---
#define SAMPLE_INTERVAL_MS  50
#define L2_LEARN_COUNT      20
#define L1_COLLECT_TOTAL    200    // 10s L1 采样

// 滤波参数 (保持 Line.c 的高稳定性设置)
#define LEN_MED_BUF         10     // 0.5s 中值
#define LEN_AVG_BUF         40     // 2.0s 滑动平均 (这也是 "Filtered CUSUM" 的精髓)

// CUSUM 特定参数 (需要根据您的 CSV 数据微调)
// Drift(k): 容忍 20 的波动 (PID 扰动大概 50，但经过滑窗后会被削减，设 20 比较安全)
// Threshold(h): 累积超过 150 判定为停止 (根据之前 Python 分析结果)
// Lambda: 0.98 遗忘因子，让偶尔的 PID 毛刺能快速消退
#define CUSUM_DRIFT_K       20.0f
#define CUSUM_THRESH_H      150.0f
#define CUSUM_LAMBDA        0.98f

// --- 内部变量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;
static uint8_t has_L1 = 0;
static uint32_t w_confirm_ms = 500; // CUSUM 也是一种积分，自带延时，但保留这个双重确认更稳

// 滤波器缓冲区
static float buf_med[LEN_MED_BUF];
static uint16_t idx_med = 0;
static float buf_avg[LEN_AVG_BUF];
static uint16_t idx_avg = 0;

// CUSUM 实例
static Cusum_t cusum_inst;

// 业务变量
static float L2_temp_max = 0.0f;
static uint16_t L2_sample_cnt = 0;
static float final_L2 = 0.0f;
static uint32_t success_start_tick = 0;
static uint8_t is_success_counting = 0;
static uint32_t predict_start_tick = 0;

// L1 变量
static uint16_t l1_collect_cnt = 0;
static float l1_temp_min = 99999.0f;

// --- 辅助函数：复用 Line.c 的强力滤波逻辑 ---
static void Reset_Buffers(void) {
    idx_med = 0; idx_avg = 0;
    memset(buf_med, 0, sizeof(buf_med));
    memset(buf_avg, 0, sizeof(buf_avg));
}

static void Push_To_Filter(float raw_val, float* med_out, uint8_t* is_valid) {
    // 1. 中值滤波
    if (idx_med < LEN_MED_BUF) buf_med[idx_med++] = raw_val;
    else {
        for(int i=0; i<LEN_MED_BUF-1; i++) buf_med[i] = buf_med[i+1];
        buf_med[LEN_MED_BUF-1] = raw_val;
    }
    if (idx_med < LEN_MED_BUF) { *is_valid = 0; return; }
    float val_median = Calc_Median(buf_med, LEN_MED_BUF);

    // 2. 滑动平均
    if (idx_avg < LEN_AVG_BUF) buf_avg[idx_avg++] = val_median;
    else {
        for(int i=0; i<LEN_AVG_BUF-1; i++) buf_avg[i] = buf_avg[i+1];
        buf_avg[LEN_AVG_BUF-1] = val_median;
    }
    if (idx_avg < LEN_AVG_BUF) { *is_valid = 0; return; }
    
    *med_out = Calc_Sliding_Average(buf_avg, LEN_AVG_BUF);
    *is_valid = 1;
}

// --- 外部接口 ---

void Line_Cusum_Init(void) {
    current_state = LINE_IDLE;
    has_L1 = 0;
    // 初始化 CUSUM 实例
    Cusum_Init(&cusum_inst, CUSUM_DRIFT_K, CUSUM_THRESH_H, CUSUM_LAMBDA);
}

Line_State_t Line_Cusum_GetState(void) { return current_state; }

void Line_Cusum_Stop(void) { current_state = LINE_IDLE; }

void Line_Cusum_Start_L1_Test(void) {
    Reset_Buffers();
    l1_collect_cnt = 0;
    l1_temp_min = 99999.0f;
    current_state = LINE_TEST_L1;
}

void Line_Cusum_Start_Work_Predict(void) {
    if (!has_L1) {
        printf("[Cusum] Error: L1 not set!\r\n");
        return;
    }
    Reset_Buffers();
    L2_sample_cnt = 0; 
    L2_temp_max = 0.0f; 
    is_success_counting = 0;
    
    Cusum_Reset(&cusum_inst); // 重置 CUSUM 状态
    
    predict_start_tick = HAL_GetTick();
    current_state = LINE_WORK_PREDICT;
}

uint8_t Line_Cusum_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    float P = 0.0f;
    uint8_t val_valid = 0;
    uint32_t current_time = HAL_GetTick();

    // 先进行滤波，压制大部分 PID 噪声
    Push_To_Filter((float)raw_adc, &P, &val_valid);

    // ============================================================
    // 1. L1 测量阶段 (保持 10s 取最小值)
    // ============================================================
    if (current_state == LINE_TEST_L1) {
        if (!val_valid) {
            printf("[%u] [Cusum] L1 Warming up... Raw:%u\r\n", current_time, raw_adc);
        } else {
            if (P < l1_temp_min) l1_temp_min = P;
            l1_collect_cnt++;
            if (l1_collect_cnt % 20 == 0) {
                printf("[%u] [Cusum] L1 Collecting (%d/%d)... P:%.2f, Raw:%u\r\n", 
                       current_time, l1_collect_cnt, L1_COLLECT_TOTAL, P, raw_adc);
            }
            if (l1_collect_cnt >= L1_COLLECT_TOTAL) {
                L1_val = l1_temp_min;
                has_L1 = 1;
                printf("--- L1 SET DONE (Min:%.2f) --- [%u] [Cusum] L1 Fixed. P:%.2f, Raw:%u\r\n", 
                       L1_val, current_time, P, raw_adc);
                current_state = LINE_IDLE;
                return 1;
            }
        }
    }
    // ============================================================
    // 2. 预测阶段 (使用 CUSUM)
    // ============================================================
    else if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
        uint32_t rel_time = current_time - predict_start_tick;

        if (!val_valid) {
            printf("[%u] Calculating... Raw:%u\r\n", rel_time, raw_adc);
            return 0;
        }

        // --- 学习 L2 阶段 ---
        if (L2_sample_cnt < L2_LEARN_COUNT) {
            if (P > L2_temp_max) L2_temp_max = P;
            L2_sample_cnt++;
            if (L2_sample_cnt >= L2_LEARN_COUNT) {
                final_L2 = L2_temp_max;
                // 学习完成，设置 CUSUM 的目标值为 L2
                Cusum_SetTarget(&cusum_inst, final_L2);
                printf("--- L2 SETTLED (L2:%.2f) --- [%u] [Cusum] Target Set. P:%.2f, Raw:%u\r\n", 
                       final_L2, rel_time, P, raw_adc);
            } else {
                printf("[%u] [Cusum] Learning L2... P:%.2f, Raw:%u\r\n", rel_time, P, raw_adc);
            }
            return 0;
        }

        // --- CUSUM 监测阶段 ---
        // 输入 P (已滤波值)，CUSUM 负责计算累积偏差 score
        float score = Cusum_Update_Decrease(&cusum_inst, P);

        if (current_state == LINE_WORK_SUCCESS) {
            printf("[%u] [Cusum] P:%.2f, Score:%.1f, [SUCCESS], Raw:%u\r\n", rel_time, P, score, raw_adc);
        } else {
            // 判定逻辑：累积和超过阈值
            if (score > cusum_inst.threshold_h) {
                if (!is_success_counting) {
                    is_success_counting = 1;
                    success_start_tick = current_time;
                }
                
                // CUSUM 已经很稳了，但为了保持逻辑一致，还是加一个短暂的确认窗
                if ((current_time - success_start_tick) >= w_confirm_ms) {
                    printf("--- WORK SUCCESS --- [%u] [Cusum] P:%.2f, Score:%.1f, [SUCCESS], Raw:%u\r\n", 
                           rel_time, P, score, raw_adc);
                    current_state = LINE_WORK_SUCCESS;
                    return 1;
                } else {
                    printf("[%u] [Cusum] P:%.2f, Score:%.1f, Pending..., Raw:%u\r\n", rel_time, P, score, raw_adc);
                }
            } else {
                is_success_counting = 0;
                printf("[%u] [Cusum] P:%.2f, Score:%.1f, Not Ready, Raw:%u\r\n", rel_time, P, score, raw_adc);
            }
        }
    }
    return 0;
}
