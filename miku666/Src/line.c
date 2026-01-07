#include "line.h"
#include "Calc_Median.h"
#include "Calc_Sliding_Average.h"
#include <stdio.h>
#include <string.h>
#include "main.h"

//L1为下线，L2为上线
// --- 參數配置 ---
#define SAMPLE_INTERVAL_MS  50    // 採樣間隔
#define PARAM_K             0.4f  // 系數 K，突变0.4，连续0.7
#define L2_LEARN_COUNT      20    // 學習 L2 需要的樣本數
#define LEN_MED_BUF         10    // 0.5s 中值濾波
#define LEN_AVG_BUF         40    // 2.0s 滑動平均

// 默认L1
#define DEFAULT_L1_VAL      50.0f

// L1 採樣參數 (10秒)
#define L1_COLLECT_DURATION_MS 10000 
#define L1_SAMPLES_TOTAL       (L1_COLLECT_DURATION_MS / SAMPLE_INTERVAL_MS)

// --- 內部變量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;       // 下線 L1
static uint8_t has_L1 = 0;        // 標記是否已測 L1
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

// 記錄預測開始的時間戳
static uint32_t predict_start_tick = 0; 

// L1 測試專用
static uint16_t l1_collect_cnt = 0;
static float l1_temp_min = 0.0f;

// --- 內部輔助函數 ---

static void Reset_Buffers(void) {
    idx_med = 0; 
    idx_avg = 0;
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

void Line_Init(void) {
    current_state = LINE_IDLE;
    has_L1 = 0;
}

Line_State_t Line_GetState(void) { return current_state; }

void Line_SetConfirmWindow(uint32_t w_ms) { w_confirm_ms = w_ms; }

// 停止預測 (刪除打印)
void Line_Stop(void) {
    current_state = LINE_IDLE;
}

// 開始 L1 (刪除打印)
void Line_Start_L1_Test(void) {
    Reset_Buffers();
    l1_collect_cnt = 0;
    l1_temp_min = 99999.0f; 
    current_state = LINE_TEST_L1;
}

void Line_Start_Work_Predict(void) {
    if (!has_L1) {
        L1_val = DEFAULT_L1_VAL;
        printf("[Line] WARNING: L1 not calibrated! Using Default: %.2f\r\n", L1_val);
        // 删除 return
    }
    Reset_Buffers();
    L2_sample_cnt = 0; 
    L2_temp_max = 0.0f; 
    is_success_counting = 0;
    predict_start_tick = HAL_GetTick(); // 重置相對時間
    current_state = LINE_WORK_PREDICT;
}

uint8_t Line_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    float P = 0.0f;
    uint8_t val_valid = 0;
    uint32_t current_time = HAL_GetTick();

    Push_To_Filter((float)raw_adc, &P, &val_valid);

    // ============================================================
    // 1. L1 測量階段
    // ============================================================
    if (current_state == LINE_TEST_L1) {
        if (!val_valid) {
            printf("[%u] L1 Warming up... Raw:%u\r\n", current_time, raw_adc);
        } else {
            if (P < l1_temp_min) l1_temp_min = P;
            l1_collect_cnt++;
            
            // 過程打印
            if (l1_collect_cnt % 20 == 0) {
                printf("[%u] L1 Collecting (%d/%d)... Curr:%.2f, Min:%.2f\r\n", 
                       current_time, l1_collect_cnt, L1_SAMPLES_TOTAL, P, l1_temp_min);
            }

            // 結束
            if (l1_collect_cnt >= L1_SAMPLES_TOTAL) {
                L1_val = l1_temp_min;
                has_L1 = 1; 
                // 將 L1 SET DONE 合併到數據行，保持格式一致
                printf("--- L1 SET DONE (Min:%.2f) --- [%u] L1 Finalizing... Curr:%.2f\r\n", 
                       L1_val, current_time, P);
                current_state = LINE_IDLE;
                return 1;
            }
        }
    }

    // ============================================================
    // 2. 預測階段
    // ============================================================
    else if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
        
        uint32_t relative_time = current_time - predict_start_tick;

        // --- 預熱 ---
        if (!val_valid) {
            printf("[%u] Calculating... Raw:%u\r\n", relative_time, raw_adc);
            return 0;
        }

        // --- 學習 L2 ---
        if (L2_sample_cnt < L2_LEARN_COUNT) {
            if (P > L2_temp_max) L2_temp_max = P;
            L2_sample_cnt++;

            // 如果是最後一個學習點，計算並打印 "L2 Settled" 到同一行
            if (L2_sample_cnt >= L2_LEARN_COUNT) {
                final_L2 = L2_temp_max;
                if (final_L2 < L1_val) {
                    printf("--- ERROR --- [Line] L2(%.2f) < L1(%.2f)!\r\n", final_L2, L1_val);
                    current_state = LINE_IDLE;
                    return 0;
                }
                alpha_threshold = PARAM_K * (final_L2 - L1_val) / (final_L2 + 0.001f);
                
                // 修改點：將 Settled 信息作為前綴合併打印
                printf("--- L2 SETTLED (L2:%.2f a:%.4f) --- [%u] Learning L2... P:%.2f, Raw:%u\r\n", 
                       final_L2, alpha_threshold, relative_time, P, raw_adc);
            } 
            else {
                // 普通學習過程打印
                printf("[%u] Learning L2... P:%.2f, Raw:%u\r\n", relative_time, P, raw_adc);
            }
            return 0;
        }

        // --- 監測 ---
        float beta = (final_L2 - P) / (final_L2 + 0.001f);

        if (current_state == LINE_WORK_SUCCESS) {
            printf("[%u] P:%.2f, beta:%.4f, [SUCCESS], Raw:%u\r\n", relative_time, P, beta, raw_adc);
        } else {
            if (beta > (alpha_threshold / 2.0f)) {
                if (!is_success_counting) { 
                    is_success_counting = 1; 
                    success_start_tick = current_time; 
                }
                
                if ((current_time - success_start_tick) >= w_confirm_ms) {
                    // 修改點：成功信息作為前綴合併打印
                    printf("--- WORK SUCCESS --- [%u] P:%.2f, beta:%.4f, [SUCCESS], Raw:%u\r\n", relative_time, P, beta, raw_adc);
                    current_state = LINE_WORK_SUCCESS;
                    return 1;
                } else {
                    printf("[%u] P:%.2f, beta:%.4f, Pending..., Raw:%u\r\n", relative_time, P, beta, raw_adc);
                }
            } else {
                is_success_counting = 0;
                printf("[%u] P:%.2f, beta:%.4f, Not Ready, Raw:%u\r\n", relative_time, P, beta, raw_adc);
            }
        }
    }
    return 0;
}
