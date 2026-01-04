#ifndef __LINE_LOW_H
#define __LINE_LOW_H

#include "main.h" // 获取uint32_t等定义

// --- 状态定义 ---
typedef enum {
    LINE_LOW_IDLE = 0, // 空闲
    LINE_LOW_BUSY,     // 正在计算
    LINE_LOW_DONE      // 完成（暂未用到，可用于扩展）
} LineLow_State_t;

// --- 函数声明 ---

// 开始测试（在按键按下时调用）
void LineLow_Start(void);

// 核心处理函数（在每次ADC采集后调用）
// 返回值：0=正在计算/无事发生, 1=计算完成并已打印
uint8_t LineLow_Process(uint32_t raw_adc);

// 获取当前状态（用于控制LED或互斥）
LineLow_State_t LineLow_GetState(void);

#endif
