#include "kalman.h"

void Kalman_Init(Kalman_t* kf, float Q, float R, float initial_value) {
    kf->x_est = initial_value;
    kf->P = 1.0f; // 初始协方差设为1
    kf->Q = Q;
    kf->R = R;
    kf->K = 0.0f;
}

float Kalman_Update(Kalman_t* kf, float z_meas) {
    // 1. 预测阶段 (Prediction)
    // 假定系统状态不变 (x_k = x_k-1)，此时 x_pred = x_est
    float P_pred = kf->P + kf->Q;

    // 2. 更新阶段 (Update)
    kf->K = P_pred / (P_pred + kf->R);             // 计算卡尔曼增益
    kf->x_est = kf->x_est + kf->K * (z_meas - kf->x_est); // 更新估计值
    kf->P = (1.0f - kf->K) * P_pred;               // 更新误差协方差

    return kf->x_est;
}
