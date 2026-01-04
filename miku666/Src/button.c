#include "button.h"

// 定义消抖时间 (ms)
#define DEBOUNCE_DELAY 50  // 50ms通常够用了，250ms稍微有点长，你可以根据手感调整

// 内部静态变量，用于记录状态，static确保它们在函数退出后数值还在
static uint32_t last_time_btn1 = 0;
static uint8_t  prev_state_btn1 = KEY_RELEASED;

static uint32_t last_time_btn2 = 0;
static uint8_t  prev_state_btn2 = KEY_RELEASED;

// 按键扫描函数
// 返回值: 0-无按键, 1-按键1按下, 2-按键2按下
uint8_t Button_Scan(void) {
    uint32_t current_time = HAL_GetTick();
    uint8_t key_return = 0; // 默认返回0

    // --- 检测按键 1 (PA3) ---
    uint8_t cur_state_btn1 = HAL_GPIO_ReadPin(BTN1_PORT, BTN1_PIN);
    
    // 如果电平发生变化（不管是从松到按，还是从按到松）
    if (cur_state_btn1 != prev_state_btn1) {
        // 检查时间差，是否满足消抖
        if ((current_time - last_time_btn1) > DEBOUNCE_DELAY) {
            // 确认状态改变有效
            if (cur_state_btn1 == KEY_PRESSED) {
                key_return = 1; // 捕捉到按键1按下的瞬间
            }
            prev_state_btn1 = cur_state_btn1; // 更新状态
            last_time_btn1 = current_time;    // 更新时间
        }
    }

    // --- 检测按键 2 (PA4) ---
    // 逻辑同上，独立的变量
    uint8_t cur_state_btn2 = HAL_GPIO_ReadPin(BTN2_PORT, BTN2_PIN);
    
    if (cur_state_btn2 != prev_state_btn2) {
        if ((current_time - last_time_btn2) > DEBOUNCE_DELAY) {
            if (cur_state_btn2 == KEY_PRESSED) {
                key_return = 2; // 捕捉到按键2按下的瞬间
            }
            prev_state_btn2 = cur_state_btn2;
            last_time_btn2 = current_time;
        }
    }

    return key_return; // 返回结果
}
