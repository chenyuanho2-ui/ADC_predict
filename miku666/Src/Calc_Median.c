#include "Calc_Median.h"

#define FILTER_MAX_LEN 200  // 定义一个内部处理的最大长度，要大于你的 MAX_SAMPLES

float Calc_Median(float* buffer, uint16_t count) {
    if (count == 0) return 0.0f;
    if (count > FILTER_MAX_LEN) count = FILTER_MAX_LEN; // 安全保护
    
    // 1. 拷贝数据到临时数组（因为我们要排序，不能动原始数据）
    float temp_buf[FILTER_MAX_LEN];
    for(int i = 0; i < count; i++) {
        temp_buf[i] = buffer[i];
    }
    
    // 2. 冒泡排序 (从小到大)
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (temp_buf[j] > temp_buf[j + 1]) {
                float temp = temp_buf[j];
                temp_buf[j] = temp_buf[j + 1];
                temp_buf[j + 1] = temp;
            }
        }
    }
    
    // 3. 取中值
    if (count % 2 == 1) {
        return temp_buf[count / 2];
    } else {
        return (temp_buf[count / 2 - 1] + temp_buf[count / 2]) / 2.0f;
    }
}
