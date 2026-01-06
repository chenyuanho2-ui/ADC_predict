#include "select.h"
#include "line.h"        // 经典算法
#include "line_kalman.h" // 卡尔曼算法
#include <stdio.h>

// 默认使用卡尔曼
static Algo_Type_t current_algo = ALGO_KALMAN_BASELINE;

void Select_SetAlgo(Algo_Type_t algo) {
    current_algo = algo;
    // 切换算法时，最好重置一下状态
    if (current_algo == ALGO_SMA_BASELINE) {
        printf("[Select] Switched to SMA Algorithm\r\n");
        Line_Init();
    } else {
        printf("[Select] Switched to Kalman Algorithm\r\n");
        Line_Kalman_Init();
    }
}

// --- 路由实现 ---

void Select_Init(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Init();
    else Line_Kalman_Init();
}

Line_State_t Select_GetState(void) {
    if (current_algo == ALGO_SMA_BASELINE) return Line_GetState();
    else return Line_Kalman_GetState();
}

void Select_Start_L1_Test(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Start_L1_Test();
    else Line_Kalman_Start_L1_Test();
}

void Select_Start_Work_Predict(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Start_Work_Predict();
    else Line_Kalman_Start_Work_Predict();
}

void Select_Stop(void) {
    if (current_algo == ALGO_SMA_BASELINE) Line_Stop(); // 需确保 line.h 中声明了 Line_Stop
    else Line_Kalman_Stop();
}

uint8_t Select_Process(uint32_t raw_adc) {
    if (current_algo == ALGO_SMA_BASELINE) return Line_Process(raw_adc);
    else return Line_Kalman_Process(raw_adc);
}
