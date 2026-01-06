#ifndef __LINE_CUSUM_H
#define __LINE_CUSUM_H

#include <stdint.h>
#include "line.h"  // 必须包含以获取 Line_State_t

void Line_Cusum_Init(void);
Line_State_t Line_Cusum_GetState(void);
void Line_Cusum_Start_L1_Test(void);
void Line_Cusum_Start_Work_Predict(void);
void Line_Cusum_Stop(void);
uint8_t Line_Cusum_Process(uint32_t raw_adc);

#endif
