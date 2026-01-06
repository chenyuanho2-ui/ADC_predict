#include "cusum.h"

void Cusum_Init(Cusum_t* cs, float drift_k, float threshold_h, float lambda) {
    cs->sum = 0.0f;
    cs->target = 0.0f;
    cs->drift_k = drift_k;
    cs->threshold_h = threshold_h;
    cs->lambda = lambda;
}

void Cusum_SetTarget(Cusum_t* cs, float target) {
    cs->target = target;
    cs->sum = 0.0f;
}

void Cusum_Reset(Cusum_t* cs) {
    cs->sum = 0.0f;
}

float Cusum_Update_Decrease(Cusum_t* cs, float input) {
    // 逻辑：检测数值是否显著低于 Target
    // 偏差 = (目标值 - 当前值) - 容忍度
    // 如果当前值跌得很多，diff 就会是正数
    float diff = (cs->target - input) - cs->drift_k;

    // 累积逻辑 (带遗忘因子)
    // S[t] = max(0, lambda * S[t-1] + diff)
    cs->sum = (cs->sum * cs->lambda) + diff;

    if (cs->sum < 0.0f) {
        cs->sum = 0.0f;
    }

    return cs->sum;
}
