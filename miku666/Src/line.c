#include "line.h"
#include "Calc_Median.h"
#include "Calc_Sliding_Average.h"
#include <stdio.h>
#include <string.h>
#include "main.h"

// --- 參數配置 ---
#define SAMPLE_INTERVAL_MS  50    // 採樣間隔
#define PARAM_K             0.4f  // 系數 K
#define L2_LEARN_COUNT      20    // 學習 L2 需要的樣本數
#define LEN_MED_BUF         10    // 0.5s 中值濾波 (10 * 50ms)
#define LEN_AVG_BUF         20    // 1.0s 滑動平均 (20 * 50ms)

// --- 內部變量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;       // 下線 L1
static uint8_t has_L1 = 0;        // 標記是否已測 L1

// 窗口變量 w (初始設置為 500ms)
static uint32_t w_confirm_ms = 500; 

// 濾波緩衝區
static float buf_med[LEN_MED_BUF];
static uint16_t idx_med = 0;
static float buf_avg[LEN_AVG_BUF];
static uint16_t idx_avg = 0;

// L2 及預測相關
static float L2_temp_max = 0.0f;
static uint16_t L2_sample_cnt = 0;
static float final_L2 = 0.0f;
static float alpha_threshold = 0.0f;
static uint32_t success_start_tick = 0;
static uint8_t is_success_counting = 0;

// L1 測試專用
static float l1_results[5];
static uint8_t l1_res_idx = 0;

// --- 內部輔助函數 ---

static void Reset_Buffers(void) {
    idx_med = 0; 
    idx_avg = 0;
    memset(buf_med, 0, sizeof(buf_med));
    memset(buf_avg, 0, sizeof(buf_avg));
}

static void Push_To_Filter(float raw_val, float* med_out, uint8_t* is_valid) {
    // 1. 中值濾波
    if (idx_med < LEN_MED_BUF) buf_med[idx_med++] = raw_val;
    else {
        for(int i=0; i<LEN_MED_BUF-1; i++) buf_med[i] = buf_med[i+1];
        buf_med[LEN_MED_BUF-1] = raw_val;
    }
    if (idx_med < LEN_MED_BUF) { *is_valid = 0; return; }
    
    float val_median = Calc_Median(buf_med, LEN_MED_BUF);

    // 2. 滑動平均濾波
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

void Line_Init(void) {
    current_state = LINE_IDLE;
    has_L1 = 0;
}

Line_State_t Line_GetState(void) { return current_state; }

void Line_SetConfirmWindow(uint32_t w_ms) { w_confirm_ms = w_ms; }

void Line_Start_L1_Test(void) {
    Reset_Buffers();
    l1_res_idx = 0;
    current_state = LINE_TEST_L1;
    printf("[Line] L1 Baseline Test Start...\r\n");
}

void Line_Start_Work_Predict(void) {
    if (!has_L1) {
        printf("[Line] Error: L1 not set! Press Btn2 first.\r\n");
        return;
    }
    Reset_Buffers();
    L2_sample_cnt = 0; 
    L2_temp_max = 0.0f; 
    is_success_counting = 0;
    current_state = LINE_WORK_PREDICT;
    printf("[Line] Work Prediction Start...\r\n");
}

uint8_t Line_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    float P = 0.0f;
    uint8_t val_valid = 0;
    uint32_t current_time = HAL_GetTick();

    Push_To_Filter((float)raw_adc, &P, &val_valid);

    // 1. L1 測量階段 (Button 2)
    if (current_state == LINE_TEST_L1) {
        if (!val_valid) {
            printf("[%lu] L1 Warming up... Raw:%lu\r\n", current_time, raw_adc);
        } else {
            // 打印 L1 計算過程中的濾波值和原始值
            printf("[%lu] L1 Proc... P:%.2f, Raw:%lu\r\n", current_time, P, raw_adc);
            if (l1_res_idx < 5) {
                l1_results[l1_res_idx++] = P;
            } else {
                float max_l1 = 0.0f;
                for(int i=0; i<5; i++) if(l1_results[i] > max_l1) max_l1 = l1_results[i];
                L1_val = max_l1;
                has_L1 = 1;
                printf("[Line] L1 Baseline Fixed at: %.2f\r\n", L1_val);
                current_state = LINE_IDLE;
                return 1;
            }
        }
    }

    // 2. 預測階段 (Button 1)
    else if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
        // --- 預熱階段 ---
        if (!val_valid) {
            printf("[%lu] Calculating... Raw:%lu\r\n", current_time, raw_adc);
            return 0;
        }

        // --- 學習 L2 階段 (前 20 個數) ---
        if (L2_sample_cnt < L2_LEARN_COUNT) {
            if (P > L2_temp_max) L2_temp_max = P;
            L2_sample_cnt++;
            // 打印: Learning L2, 濾波值P, 原始值
            printf("[%lu] Learning L2... P:%.2f, Raw:%lu\r\n", current_time, P, raw_adc);
            
            if (L2_sample_cnt >= L2_LEARN_COUNT) {
                final_L2 = L2_temp_max;
                // 報錯檢查: 如果 L2 < L1
                if (final_L2 < L1_val) {
                    printf("[Line] FATAL ERROR: L2(%.2f) < L1(%.2f)! Baseline anomaly.\r\n", final_L2, L1_val);
                    current_state = LINE_IDLE; // 強制終止
                    return 0;
                }
                // 計算 alpha
                alpha_threshold = PARAM_K * (final_L2 - L1_val) / (final_L2 + 0.001f);
                printf("[Line] L2 Settled: %.2f, alpha: %.4f\r\n", final_L2, alpha_threshold);
            }
            return 0;
        }

        // --- 監測階段 ---
        float beta = (final_L2 - P) / (final_L2 + 0.001f);

        if (current_state == LINE_WORK_SUCCESS) {
            // 成功後的持續打印: P, beta, success, Raw
            printf("[%lu] P:%.2f, beta:%.4f, [SUCCESS], Raw:%lu\r\n", current_time, P, beta, raw_adc);
        } else {
            // 判定邏輯: beta > alpha / 2
            if (beta > (alpha_threshold / 2.0f)) {
                if (!is_success_counting) { 
                    is_success_counting = 1; 
                    success_start_tick = current_time; 
                }
                
                // 檢查是否維持了 w (w_confirm_ms)
                if ((current_time - success_start_tick) >= w_confirm_ms) {
                    printf("[%lu] P:%.2f, beta:%.4f, [SUCCESS], Raw:%lu\r\n", current_time, P, beta, raw_adc);
                    current_state = LINE_WORK_SUCCESS;
                    return 1;
                } else {
                    // 打印: P, beta, Pending, Raw
                    printf("[%lu] P:%.2f, beta:%.4f, Pending..., Raw:%lu\r\n", current_time, P, beta, raw_adc);
                }
            } else {
                // 不滿足條件，重置計時
                is_success_counting = 0;
                // 打印: P, beta, 未成功, Raw
                printf("[%lu] P:%.2f, beta:%.4f, Not Ready, Raw:%lu\r\n", current_time, P, beta, raw_adc);
            }
        }
    }
    return 0;
}
