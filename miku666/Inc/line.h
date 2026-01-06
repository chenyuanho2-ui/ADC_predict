#ifndef __LINE_H
#define __LINE_H

#include <stdint.h>

// 1. 基础状态枚举定义 (所有算法共用)
typedef enum {
    LINE_IDLE = 0,
    LINE_TEST_L1,
    LINE_WORK_PREDICT,
    LINE_WORK_SUCCESS
} Line_State_t;

// 2. 经典 SMA 算法的函数声明
void Line_Init(void);
Line_State_t Line_GetState(void);
void Line_Start_L1_Test(void);
void Line_Start_Work_Predict(void);
void Line_Stop(void);
uint8_t Line_Process(uint32_t raw_adc);

#endif
