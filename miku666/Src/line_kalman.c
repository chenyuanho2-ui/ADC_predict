#include "line_kalman.h"
#include "kalman.h"
#include "main.h"
#include <stdio.h>

// --- 参数配置 ---
// 卡尔曼参数
#define KALMAN_Q            0.01f  // 过程噪声 (根据之前分析推荐)
#define KALMAN_R            0.5f   // 测量噪声 (抵抗PID突起)

// 业务参数 (保持与 line.c 一致)
#define PARAM_K             0.4f
#define L2_LEARN_COUNT      20
#define L1_COLLECT_TOTAL    200    // 10s @ 50ms

// --- 内部变量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;
static uint8_t has_L1 = 0;
static uint32_t w_confirm_ms = 500;

// 卡尔曼滤波器实例
static Kalman_t kf_inst;
static uint8_t kf_initialized = 0; // 标记滤波器是否已重置

// L2 及预测相关
static float L2_temp_max = 0.0f;
static uint16_t L2_sample_cnt = 0;
static float final_L2 = 0.0f;
static float alpha_threshold = 0.0f;
static uint32_t success_start_tick = 0;
static uint8_t is_success_counting = 0;
static uint32_t predict_start_tick = 0;

// L1 测试专用
static uint16_t l1_collect_cnt = 0;
static float l1_temp_min = 99999.0f;

// --- 外部接口实现 ---

void Line_Kalman_Init(void) {
    current_state = LINE_IDLE;
    has_L1 = 0;
    kf_initialized = 0;
}

Line_State_t Line_Kalman_GetState(void) {
    return current_state;
}

void Line_Kalman_Stop(void) {
    current_state = LINE_IDLE;
}

void Line_Kalman_Start_L1_Test(void) {
    l1_collect_cnt = 0;
    l1_temp_min = 99999.0f;
    kf_initialized = 0; // 重新初始化滤波器
    current_state = LINE_TEST_L1;
}

void Line_Kalman_Start_Work_Predict(void) {
    if (!has_L1) {
        printf("[Kalman] Error: L1 not set!\r\n");
        return;
    }
    L2_sample_cnt = 0;
    L2_temp_max = 0.0f;
    is_success_counting = 0;
    kf_initialized = 0; // 重新初始化滤波器，避免上一段数据的拖尾
    predict_start_tick = HAL_GetTick();
    current_state = LINE_WORK_PREDICT;
}

uint8_t Line_Kalman_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    uint32_t current_time = HAL_GetTick();
    float raw_f = (float)raw_adc;
    float P = 0.0f;

    // 1. 卡尔曼滤波处理
    // 如果是该状态下的第一个点，初始化滤波器
    if (!kf_initialized) {
        Kalman_Init(&kf_inst, KALMAN_Q, KALMAN_R, raw_f);
        kf_initialized = 1;
        P = raw_f; // 第一个点直接作为估计值
    } else {
        P = Kalman_Update(&kf_inst, raw_f);
    }

    // ============================================================
    // 1. L1 测量阶段
    // ============================================================
    if (current_state == LINE_TEST_L1) {
        if (P < l1_temp_min) l1_temp_min = P;
        l1_collect_cnt++;

        if (l1_collect_cnt % 20 == 0) {
            printf("[%u] [Kalman] L1 Collecting (%d/%d)... P:%.2f\r\n", 
                   current_time, l1_collect_cnt, L1_COLLECT_TOTAL, P);
        }

        if (l1_collect_cnt >= L1_COLLECT_TOTAL) {
            L1_val = l1_temp_min;
            has_L1 = 1;
            printf("--- L1 DONE (Min:%.2f) --- [Kalman] L1 Fixed.\r\n", L1_val);
            current_state = LINE_IDLE;
            return 1;
        }
    }
    // ============================================================
    // 2. 预测阶段
    // ============================================================
    else if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
        uint32_t rel_time = current_time - predict_start_tick;

        // --- 学习 L2 ---
        if (L2_sample_cnt < L2_LEARN_COUNT) {
            if (P > L2_temp_max) L2_temp_max = P;
            L2_sample_cnt++;
            
            if (L2_sample_cnt >= L2_LEARN_COUNT) {
                final_L2 = L2_temp_max;
                alpha_threshold = PARAM_K * (final_L2 - L1_val) / (final_L2 + 0.001f);
                printf("--- L2 SETTLED (%.2f) --- [Kalman] alpha:%.4f\r\n", final_L2, alpha_threshold);
            } else {
                printf("[%u] [Kalman] Learning L2... P:%.2f\r\n", rel_time, P);
            }
            return 0;
        }

        // --- 监测阶段 ---
        float beta = (final_L2 - P) / (final_L2 + 0.001f);

        if (current_state == LINE_WORK_SUCCESS) {
            printf("[%u] [Kalman] P:%.2f, beta:%.4f [SUCCESS]\r\n", rel_time, P, beta);
        } else {
            // 判定逻辑
            if (beta > (alpha_threshold / 2.0f)) {
                if (!is_success_counting) {
                    is_success_counting = 1;
                    success_start_tick = current_time;
                }
                
                if ((current_time - success_start_tick) >= w_confirm_ms) {
                    printf("--- WORK SUCCESS --- [%u] [Kalman] P:%.2f, beta:%.4f\r\n", rel_time, P, beta);
                    current_state = LINE_WORK_SUCCESS;
                    return 1;
                } else {
                    printf("[%u] [Kalman] P:%.2f, beta:%.4f Pending...\r\n", rel_time, P, beta);
                }
            } else {
                is_success_counting = 0;
                printf("[%u] [Kalman] P:%.2f, beta:%.4f Not Ready\r\n", rel_time, P, beta);
            }
        }
    }
    return 0;
}
