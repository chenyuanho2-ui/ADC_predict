#ifndef __CUSUM_H
#define __CUSUM_H

#include <stdint.h>

typedef struct {
    float sum;          // 当前累积和 (Cumulative Sum)
    float target;       // 目标均值 (通常是 L2 基准)
    float drift_k;      // 漂移容忍度 (k): 只有偏差超过 k 才会开始累积
    float threshold_h;  // 报警阈值 (h): 累积和超过此值判定为异常
    float lambda;       // 遗忘因子 (0.0 ~ 1.0): 1.0 为标准 CUSUM，<1.0 (如 0.95) 可让旧误差衰减
} Cusum_t;

// 初始化
// lambda 建议设为 0.95 ~ 0.99 以抑制长期误报，设为 1.0 则为标准累积
void Cusum_Init(Cusum_t* cs, float drift_k, float threshold_h, float lambda);

// 设置目标值 (通常在 L2 学习完成后调用)
void Cusum_SetTarget(Cusum_t* cs, float target);

// 复位累积和
void Cusum_Reset(Cusum_t* cs);

// 更新计算 (检测数值下降的情况)
// input: 当前的滤波后数值
// 返回值: 当前的累积评分 sum
float Cusum_Update_Decrease(Cusum_t* cs, float input);

#endif
