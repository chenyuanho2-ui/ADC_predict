#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>

typedef struct {
    float x_est;  // 估计值 (Estimate)
    float P;      // 估计协方差 (Error Covariance)
    float Q;      // 过程噪声 (Process Noise) - 调小：更平滑，滞后增加
    float R;      // 测量噪声 (Measurement Noise) - 调大：更抗干扰，对突变不敏感
    float K;      // 卡尔曼增益 (Kalman Gain)
} Kalman_t;

// 初始化
void Kalman_Init(Kalman_t* kf, float Q, float R, float initial_value);

// 更新计算
// z_meas: 当前测量值 (Raw ADC)
// 返回值: 滤波后的估计值
float Kalman_Update(Kalman_t* kf, float z_meas);

#endif
