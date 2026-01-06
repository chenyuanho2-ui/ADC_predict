#ifndef __LINE_KALMAN_H
#define __LINE_KALMAN_H

#include "line.h" // 复用 line.h 中的 Line_State_t 定义

// 初始化
void Line_Kalman_Init(void);

// 获取状态
Line_State_t Line_Kalman_GetState(void);

// 控制接口
void Line_Kalman_Start_L1_Test(void);
void Line_Kalman_Start_Work_Predict(void);
void Line_Kalman_Stop(void);

// 核心处理 (替代原有的 Line_Process)
uint8_t Line_Kalman_Process(uint32_t raw_adc);

#endif
