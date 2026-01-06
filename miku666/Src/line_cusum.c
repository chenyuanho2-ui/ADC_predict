#include "line_cusum.h"
#include "cusum.h"
#include "Calc_Median.h"
#include "Calc_Sliding_Average.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h> // 用于 fabs 或其他数学计算(如果需要)

// --- 参数配置 ---
#define SAMPLE_INTERVAL_MS  50
#define L2_LEARN_COUNT      20
#define L1_COLLECT_TOTAL    200    // 10s L1 采样

// 滤波参数
#define LEN_MED_BUF         10     // 0.5s 中值
#define LEN_AVG_BUF         40     // 2.0s 滑动平均

// --- CUSUM 自适应参数 (关键修改) ---
// 不再使用固定的数值，而是使用"比例"
// K_RATIO: 容忍度占量程的比例 (推荐 0.15 即 15%)
// H_RATIO: 阈值占量程的比例 (推荐 0.8 ~ 1.0)
#define CUSUM_K_RATIO       0.15f
#define CUSUM_H_RATIO       1.0f
#define CUSUM_LAMBDA        0.98f  // 遗忘因子保持不变

// --- 内部变量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;
static uint8_t has_L1 = 0;
static uint32_t w_confirm_ms = 500; 

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

// --- 辅助函数 ---
static void Reset_Buffers(void) {
    idx_med = 0; idx_avg = 0;
    memset(buf_med, 0, sizeof(buf_med));
    memset(buf_avg, 0, sizeof(buf_avg));
}

static void Push_To_Filter(float raw_val, float* med_out, uint8_t* is_valid) {
    if (idx_med < LEN_MED_BUF) buf_med[idx_med++] = raw_val;
    else {
        for(int i=0; i<LEN_MED_BUF-1; i++) buf_med[i] = buf_med[i+1];
        buf_med[LEN_MED_BUF-1] = raw_val;
    }
    if (idx_med < LEN_MED_BUF) { *is_valid = 0; return; }
    float val_median = Calc_Median(buf_med, LEN_MED_BUF);

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
    // 初始化时先给个默认值，防止未开始就调用 Process 出错
    Cusum_Init(&cusum_inst, 20.0f, 100.0f, CUSUM_LAMBDA);
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
    
    Cusum_Reset(&cusum_inst);
    
    predict_start_tick = HAL_GetTick();
    current_state = LINE_WORK_PREDICT;
}

uint8_t Line_Cusum_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    float P = 0.0f;
    uint8_t val_valid = 0;
    uint32_t current_time = HAL_GetTick();

    Push_To_Filter((float)raw_adc, &P, &val_valid);

    // ============================================================
    // 1. L1 测量阶段
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
    // 2. 预测阶段
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
                
                // === 关键改进：检查 L2 和 L1 的关系 ===
                if (final_L2 <= L1_val + 5.0f) {
                    printf("--- ERROR --- [Cusum] L2(%.2f) too close to L1(%.2f)! Check Hardware.\r\n", final_L2, L1_val);
                    current_state = LINE_IDLE;
                    return 0;
                }

                // === 关键改进：利用 L1 计算自适应参数 ===
                float range = final_L2 - L1_val;
                
                // 1. 计算漂移容忍度 K (量程的 15%)
                // 只有当信号跌幅超过这个值时，CUSUM 才开始计数
                float adaptive_k = range * CUSUM_K_RATIO;
                
                // 2. 计算报警阈值 H (量程的 100%)
                // 相当于积分面积达到一定程度
                float adaptive_h = range * CUSUM_H_RATIO;

                // 重新初始化 CUSUM 参数
                Cusum_Init(&cusum_inst, adaptive_k, adaptive_h, CUSUM_LAMBDA);
                Cusum_SetTarget(&cusum_inst, final_L2);

                printf("--- L2 SETTLED (L2:%.2f L1:%.2f) --- [%u] [Cusum] Range:%.1f, K:%.1f, H:%.1f, Raw:%u\r\n", 
                       final_L2, L1_val, rel_time, range, adaptive_k, adaptive_h, raw_adc);
                       
            } else {
                printf("[%u] [Cusum] Learning L2... P:%.2f, Raw:%u\r\n", rel_time, P, raw_adc);
            }
            return 0;
        }

        // --- CUSUM 监测阶段 ---
        float score = Cusum_Update_Decrease(&cusum_inst, P);

        if (current_state == LINE_WORK_SUCCESS) {
            printf("[%u] [Cusum] P:%.2f, Score:%.1f, [SUCCESS], Raw:%u\r\n", rel_time, P, score, raw_adc);
        } else {
            // 判定逻辑
            if (score > cusum_inst.threshold_h) {
                if (!is_success_counting) {
                    is_success_counting = 1;
                    success_start_tick = current_time;
                }
                
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
