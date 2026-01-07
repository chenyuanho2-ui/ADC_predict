#include "line_kalman.h"
#include "kalman.h"
#include "main.h"
#include <stdio.h>

// 卡尔曼参数 (Q=0.01, R=0.5 是初始推荐值，可根据 PID 扰动实测调整 R 到 1.0~5.0)
//Q代表系状态变化的频率,R代表测量仪器的精度（即传感器的噪声）
#define KALMAN_Q            0.01f  
#define KALMAN_R            20.0f  //注意.0不能去掉 
// 默认 L1 值
#define DEFAULT_L1_VAL      50.0f

// 业务参数
#define PARAM_K             0.4f
#define L2_SKIP_COUNT       5      // 跳过前 0.25s，让滤波器收敛并躲开启动尖刺
#define L2_LEARN_COUNT      20     // 学习 1.0s 的最大值作为上线
#define L1_COLLECT_TOTAL    200    // 10s @ 50ms

// --- 内部变量 ---
static Line_State_t current_state = LINE_IDLE;
static float L1_val = 0.0f;
static uint8_t has_L1 = 0;
static uint32_t w_confirm_ms = 500;//500ms

// 卡尔曼滤波器实例
static Kalman_t kf_inst;
static uint8_t kf_initialized = 0; 

// L2 及预测相关变量
static float L2_temp_max = 0.0f;
static uint16_t L2_sample_cnt = 0;
static float final_L2 = 0.0f;
static float alpha_threshold = 0.0f;
static uint32_t success_start_tick = 0;
static uint8_t is_success_counting = 0;
static uint32_t predict_start_tick = 0;

// L1 测试专用变量
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
    kf_initialized = 0; 
    current_state = LINE_TEST_L1;
}

void Line_Kalman_Start_Work_Predict(void) {
    // 【修改】不再报错返回，而是使用默认值警告
    if (!has_L1) {
        L1_val = DEFAULT_L1_VAL;
        printf("[Kalman] WARNING:Using Default: %.2f\r\n", L1_val);
    }

    L2_sample_cnt = 0;
    L2_temp_max = 0.0f;
    is_success_counting = 0;
    kf_initialized = 0; 
    predict_start_tick = HAL_GetTick();
    current_state = LINE_WORK_PREDICT;
}

/**
 * @brief 卡尔曼基线核心处理函数
 */
uint8_t Line_Kalman_Process(uint32_t raw_adc) {
    if (current_state == LINE_IDLE) return 0;

    uint32_t current_time = HAL_GetTick();
    float raw_f = (float)raw_adc;
    float P = 0.0f;

    // 1. 运行卡尔曼滤波
    if (!kf_initialized) {
        Kalman_Init(&kf_inst, KALMAN_Q, KALMAN_R, raw_f);
        kf_initialized = 1;
        P = raw_f; 
    } else {
        P = Kalman_Update(&kf_inst, raw_f);
    }

    // ============================================================
    // 阶段 1: L1 测量阶段
    // ============================================================
    if (current_state == LINE_TEST_L1) {
        if (P < l1_temp_min) l1_temp_min = P;
        l1_collect_cnt++;

        if (l1_collect_cnt % 20 == 0) {
            printf("[%u] [Kalman] L1 Collecting (%d/%d)... P:%.2f, Raw:%u\r\n", 
                   current_time, l1_collect_cnt, L1_COLLECT_TOTAL, P, raw_adc);
        }

        if (l1_collect_cnt >= L1_COLLECT_TOTAL) {
            L1_val = l1_temp_min;
            has_L1 = 1;
            printf("--- L1 SET DONE (Min:%.2f) --- [%u] P:%.2f, Raw:%u\r\n", 
                   L1_val, current_time, P, raw_adc);
            current_state = LINE_IDLE;
            return 1;
        }
    }
    // ============================================================
    // 阶段 2: 预测阶段 (包含稳定期 + 学习期 + 监测期)
    // ============================================================
    else if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
        uint32_t rel_time = current_time - predict_start_tick;
        uint16_t total_init_count = L2_SKIP_COUNT + L2_LEARN_COUNT;

        // --- A. L2 初始化 (跳过尖刺并寻找最大值) ---
        if (L2_sample_cnt < total_init_count) {
            L2_sample_cnt++;

            // 跳过阶段
            if (L2_sample_cnt <= L2_SKIP_COUNT) {
                L2_temp_max = 0.0f; 
                printf("[%u] [Kalman] Stabilizing (%d/%d)... P:%.2f, Raw:%u\r\n", 
                       rel_time, L2_sample_cnt, L2_SKIP_COUNT, P, raw_adc);
            }
            // 正式学习阶段
            else {
                if (P > L2_temp_max) L2_temp_max = P;
                
                if (L2_sample_cnt >= total_init_count) {
                    final_L2 = L2_temp_max;
                    
                    if (final_L2 <= L1_val + 5.0f) {
                         printf("--- ERROR --- L2(%.2f) too close to L1(%.2f). IDLE. Raw:%u\r\n", 
                                final_L2, L1_val, raw_adc);
                         current_state = LINE_IDLE;
                         return 0;
                    }

                    alpha_threshold = PARAM_K * (final_L2 - L1_val) / (final_L2 + 0.001f);
                    printf("--- L2 SETTLED (L2:%.2f a:%.4f) --- [%u] Ready. P:%.2f, Raw:%u\r\n", 
                           final_L2, alpha_threshold, rel_time, P, raw_adc);
                } else {
                    printf("[%u] [Kalman] Learning L2 (%d/%d)... P:%.2f, Max:%.2f, Raw:%u\r\n", 
                           rel_time, (L2_sample_cnt - L2_SKIP_COUNT), L2_LEARN_COUNT, P, L2_temp_max, raw_adc);
                }
            }
            return 0;
        }

        // --- B. 正常监测阶段 ---
        float beta = (final_L2 - P) / (final_L2 + 0.001f);

        if (current_state == LINE_WORK_SUCCESS) {
            printf("[%u] [Kalman] P:%.2f, beta:%.4f, [SUCCESS], Raw:%u\r\n", rel_time, P, beta, raw_adc);
        } else {
            if (beta > (alpha_threshold / 2.0f)) {
                if (!is_success_counting) {
                    is_success_counting = 1;
                    success_start_tick = current_time;
                }
                
                if ((current_time - success_start_tick) >= w_confirm_ms) {
                    printf("--- WORK SUCCESS --- [%u] [Kalman] P:%.2f, beta:%.4f, [SUCCESS], Raw:%u\r\n", 
                           rel_time, P, beta, raw_adc);
                    current_state = LINE_WORK_SUCCESS;
                    return 1;
                } else {
                    printf("[%u] [Kalman] P:%.2f, beta:%.4f, Pending..., Raw:%u\r\n", rel_time, P, beta, raw_adc);
                }
            } else {
                is_success_counting = 0;
                printf("[%u] [Kalman] P:%.2f, beta:%.4f, Not Ready, Raw:%u\r\n", rel_time, P, beta, raw_adc);
            }
        }
    }
    return 0;
}
