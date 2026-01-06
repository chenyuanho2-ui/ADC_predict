#ifndef __SELECT_H
#define __SELECT_H

#include <stdint.h>
#include "line.h" // 包含 Line_State_t 定义

// 算法枚举
typedef enum {
    ALGO_SMA_BASELINE = 0, // 原有的滑动平均基线法 (line.c)
    ALGO_KALMAN_BASELINE   // 新的卡尔曼基线法 (line_kalman.c)
} Algo_Type_t;

// --- 设置接口 ---
// 选择当前使用的算法
void Select_SetAlgo(Algo_Type_t algo);

// --- 统一封装接口 (Main函数调用这些) ---
// 下面这些函数会自动路由到对应的算法实现 (line.c 或 line_kalman.c)

void Select_Init(void);
Line_State_t Select_GetState(void);

void Select_Start_L1_Test(void);
void Select_Start_Work_Predict(void);
void Select_Stop(void);

uint8_t Select_Process(uint32_t raw_adc);

#endif
