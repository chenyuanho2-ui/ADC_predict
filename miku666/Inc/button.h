#ifndef __BUTTON_H
#define __BUTTON_H

#include "main.h" // 引入HAL库，以便识别GPIO定义

// --- 硬件定义 ---
// BUTTON1 (原 PA3)
#define BTN1_PIN   GPIO_PIN_3
#define BTN1_PORT  GPIOA

// BUTTON2 (新 PA4)
#define BTN2_PIN   GPIO_PIN_4
#define BTN2_PORT  GPIOA

// 按键状态定义 (上拉输入：按下是低电平0，松开是高电平1)
#define KEY_PRESSED  0
#define KEY_RELEASED 1

// --- 函数声明 ---
// 扫描按键，返回按下的键值：0=无动作, 1=按键1, 2=按键2
uint8_t Button_Scan(void);

#endif
