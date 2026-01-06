#ifndef __LINE_H
#define __LINE_H

#include <stdint.h>

// 模块状态枚举
typedef enum {
    LINE_IDLE = 0,      // 空闲
    LINE_TEST_L1,       // 正在测下线 L1
    LINE_WORK_PREDICT,  // 正在进行工作预测
    LINE_WORK_SUCCESS   // 工作完成（预测成功）
} Line_State_t;

// 初始化
void Line_Init(void);

// 获取当前模块状态
Line_State_t Line_GetState(void);

// 启动命令
void Line_Start_L1_Test(void);
void Line_Start_Work_Predict(void);

// 停止命令 (新增)
void Line_Stop(void);

// 修改判定窗口时间 w (单位: ms)
void Line_SetConfirmWindow(uint32_t w_ms);

// 核心处理函数
uint8_t Line_Process(uint32_t raw_adc);

#endif
