/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  
#include <inttypes.h>  
#include "button.h"
#include "line.h"
#include "select.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_PIN GPIO_PIN_3  // 改为PA3引脚
#define KEY_GPIO_PORT GPIOA
#define KEY_PRESSED 0  // 上拉输入时，按键按下为低电平（0）
#define STOP_DELAY 1000  // 停止前的延迟时间(ms)


#define LED0_PIN GPIO_PIN_13
#define LED0_GPIO_PORT GPIOC
#define LED0_ON  HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_PIN, GPIO_PIN_RESET)  // 假设低电平亮（根据硬件确定）
#define LED0_OFF HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_PIN, GPIO_PIN_SET)    // 高电平灭
#define LED0_TOGGLE HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_PIN)  // 翻转状态



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//	uint32_t adc_value;
//    float voltage;
//	uint32_t current_time;  // 用于记录当前时间戳（毫秒）
//	uint32_t tick_count = 0;  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1);  
	
// 初始化选择器 
	Select_Init();

   printf("symtem on...\r\n");
   
   LED0_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 初始化“下一次采样目标时间”为当前时间
  uint32_t target_tick = HAL_GetTick();
  const uint32_t SAMPLE_PERIOD = 50; // 严格的 50ms 周期
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// ============================================================
	while (HAL_GetTick() < target_tick) {
        // 在等待期间持续扫描按键，响应更快
        uint8_t key = Button_Scan();
        Line_State_t current_state = Select_GetState();

        if (key == 1) { 
            if (current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) {
                Select_Stop(); 
            } else if (current_state == LINE_IDLE) {
                // 重置时间锚点，防止暂停期间 target_tick 落后太久导致瞬间连发
                target_tick = HAL_GetTick(); 
                Select_Start_Work_Predict();
            }
        }
        else if (key == 2) {
            if (current_state == LINE_IDLE || current_state == LINE_WORK_SUCCESS) {
                target_tick = HAL_GetTick(); // 重置时间锚点
                Select_Start_L1_Test();
            }
        }
    }

    // ============================================================
    // 2. 到达时间点，执行核心任务
    // ============================================================
    // 锁定下一次的任务时间 (强制 +50，消除累积漂移)
    target_tick += SAMPLE_PERIOD;

    Line_State_t current_state = Select_GetState();

    // 只有在工作状态下才执行 ADC 采样和算法
    if (current_state == LINE_TEST_L1 || current_state == LINE_WORK_PREDICT || current_state == LINE_WORK_SUCCESS) 
    {
        // --- LED 控制 ---
        if (current_state == LINE_WORK_SUCCESS) {
            static uint8_t blink_cnt = 0;
            blink_cnt++;
            if (blink_cnt >= 2) { // 100ms 闪烁
                LED0_TOGGLE;
                blink_cnt = 0;
            }
        } else {
            LED0_ON; 
        }

        // --- ADC 采样与算法处理 ---
        // 此时已经是严格的 50ms 间隔点
        HAL_ADC_Start(&hadc1); 
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
            Select_Process(adc_val);
        }
    }
    else 
    {
        // 空闲状态：不需要严格对齐，让出 CPU 防止死循环过热（虽然 MCU 不怕热）
        LED0_OFF;
        HAL_Delay(10); // 稍微休眠一下
        // 空闲时要不断把 target_tick 同步到当前，否则一旦启动会因为 target 滞后而疯狂补作业
        target_tick = HAL_GetTick() + SAMPLE_PERIOD;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  __disable_irq();
  uint32_t timeout = 0;
  while (1)
  {
    timeout++;
    if (timeout > 2000000) // Լ1�볬ʱ��λ���������Ƶ������
    {
      NVIC_SystemReset(); // �����λ
    }
  }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
