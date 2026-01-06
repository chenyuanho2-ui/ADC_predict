#include "select.h"
#include "line.h"
#include "line_kalman.h"
#include "line_cusum.h" // 新增
#include <stdio.h>

/* * ============================================================
 * 算法选择说明 (ALGO SELECTION GUIDE):
 * ============================================================
 * 如果需要更改系统默认运行的算法，请修改下方 current_algo 的赋值：
 * * 1. ALGO_SMA_BASELINE   : 经典滑动平均法。适合处理随机高频毛刺，逻辑简单。
 * 2. ALGO_KALMAN_BASELINE : 卡尔曼滤波法。响应速度最快，能平衡噪声与灵敏度。
 * 3. ALGO_CUSUM_BASELINE  : 滤波累积和法。最稳健，适合处理 PID 带来的矩形突起干扰。
 * ============================================================
 */
static Algo_Type_t current_algo = ALGO_SMA_BASELINE; 

void Select_SetAlgo(Algo_Type_t algo) {
    current_algo = algo;
    if (current_algo == ALGO_SMA_BASELINE) {
        printf("[Select] Switched to SMA Baseline\r\n");
        Line_Init();
    } else if (current_algo == ALGO_KALMAN_BASELINE) {
        printf("[Select] Switched to Kalman Baseline\r\n");
        Line_Kalman_Init();
    } else {
        printf("[Select] Switched to Filtered CUSUM\r\n");
        Line_Cusum_Init();
    }
}

void Select_Init(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Init();
    else if (current_algo == ALGO_KALMAN_BASELINE) Line_Kalman_Init();
    else Line_Cusum_Init();
}

Line_State_t Select_GetState(void) {
    if (current_algo == ALGO_SMA_BASELINE) return Line_GetState();
    else if (current_algo == ALGO_KALMAN_BASELINE) return Line_Kalman_GetState();
    else return Line_Cusum_GetState();
}

void Select_Start_L1_Test(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Start_L1_Test();
    else if (current_algo == ALGO_KALMAN_BASELINE) Line_Kalman_Start_L1_Test();
    else Line_Cusum_Start_L1_Test();
}

void Select_Start_Work_Predict(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Start_Work_Predict();
    else if (current_algo == ALGO_KALMAN_BASELINE) Line_Kalman_Start_Work_Predict();
    else Line_Cusum_Start_Work_Predict();
}

void Select_Stop(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Stop();
    else if (current_algo == ALGO_KALMAN_BASELINE) Line_Kalman_Stop();
    else Line_Cusum_Stop();
}

uint8_t Select_Process(uint32_t raw_adc) {
    if (current_algo == ALGO_SMA_BASELINE) return Line_Process(raw_adc);
    else if (current_algo == ALGO_KALMAN_BASELINE) return Line_Kalman_Process(raw_adc);
    else return Line_Cusum_Process(raw_adc);
}
