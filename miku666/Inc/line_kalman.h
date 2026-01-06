#ifndef __LINE_KALMAN_H
#define __LINE_KALMAN_H

#include <stdint.h>
#include "line.h"  // 必须包含以获取 Line_State_t

void Line_Kalman_Init(void);
Line_State_t Line_Kalman_GetState(void);
void Line_Kalman_Start_L1_Test(void);
void Line_Kalman_Start_Work_Predict(void);
void Line_Kalman_Stop(void);
uint8_t Line_Kalman_Process(uint32_t raw_adc);

#endif
