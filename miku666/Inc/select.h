#ifndef __SELECT_H
#define __SELECT_H

#include <stdint.h>
#include "line.h"        // 获取 Line_State_t
#include "line_kalman.h" // 获取卡尔曼相关声明
#include "line_cusum.h"  // 关键：必须包含 CUSUM 的头文件

// 算法枚举定义
typedef enum {
    ALGO_SMA_BASELINE = 0,    // 经典滑动平均
    ALGO_KALMAN_BASELINE,     // 卡尔曼滤波
    ALGO_CUSUM_BASELINE       // 报错点：确保这一行存在且拼写正确
} Algo_Type_t;

// 选择器接口
void Select_SetAlgo(Algo_Type_t algo);
void Select_Init(void);
Line_State_t Select_GetState(void);
void Select_Start_L1_Test(void);
void Select_Start_Work_Predict(void);
void Select_Stop(void);
uint8_t Select_Process(uint32_t raw_adc);

#endif
