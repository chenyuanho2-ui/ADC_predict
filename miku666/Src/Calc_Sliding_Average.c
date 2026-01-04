#include "Calc_Sliding_Average.h"

float Calc_Sliding_Average(float* buffer, uint16_t count) {
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    return sum / count;
}
