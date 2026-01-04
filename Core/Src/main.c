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
#include "Calc_Sliding_Average.h"
#include "Calc_Median.h"
#include "line_low.h" 
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
uint8_t measure_state = 0;  // 0: 未测量 1: 正在测量
uint8_t stop_flag = 0;      // 0: 正常运行 1: 等待停止（取消测量后进入该状态）
uint32_t stop_start_time = 0;  // 记录开始倒计时的时间（用于计算1秒延迟）
uint32_t start_count = 0;    // 新增：记录启动读数的次数
uint32_t measure_start_time = 0;  // 新增：记录本次测量的启动时间（系统绝对时间）


// --- 新增：滤波相关变量 ---
#define MAX_SAMPLES 60       // 最大缓存点数 (足够存2秒数据即可)
#define WINDOW_MS 2000       // 窗口时间 2000ms (2秒)

float buf_val[MAX_SAMPLES];  // 存放ADC原始值的缓存区
uint32_t buf_time[MAX_SAMPLES]; // 存放对应时间戳的缓存区
uint16_t buf_count = 0;      // 当前缓存了多少个数据

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
	uint32_t adc_value;
//    float voltage;
	uint32_t current_time;  // 用于记录当前时间戳（毫秒）
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
 HAL_ADC_Start(&hadc1);  // ����ADC
 
 

   printf("symtem on...\r\n");
   
   LED0_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 1. 扫描按键 (需要在 main.c 开头引入 "button.h")
  uint8_t key_val = Button_Scan(); 

  // --- 按键逻辑处理 ---
  
  // 逻辑 A: 按下 Button 1 (PA3) -> 控制普通测量 (带延时停止)
  if (key_val == 1) 
  {
      // 互斥锁：只有当下线测试空闲时，才允许操作普通测量
      if (LineLow_GetState() == LINE_LOW_IDLE) {
          if(measure_state) {
              // 正在测量 -> 切换到等待停止
              measure_state = 0;       
              stop_flag = 1;           
              stop_start_time = HAL_GetTick();  
          } else {
              // 没测量 -> 开始测量
              measure_state = 1;       
              stop_flag = 0;           
              start_count++;  
              measure_start_time = HAL_GetTick(); // 记录起点
              
              // 每次新开始，最好清空一下之前的缓存
              buf_count = 0;
          }
      } else {
           printf("[System] Busy in LineLow Test! Ignoring Button 1.\r\n");
      }
  }
  // 逻辑 B: 按下 Button 2 (PA4) -> 启动下线测试 (LineLow)
  else if (key_val == 2) 
  {
      // 互斥锁：只有当普通测量完全停止，且下线测试也没在跑的时候，才允许启动
      if (!measure_state && !stop_flag && LineLow_GetState() == LINE_LOW_IDLE) {
          LineLow_Start(); // 启动下线测试状态机
      } else {
          printf("[System] System Busy! Cannot start LineLow Test.\r\n");
      }
  }


  // --- 核心执行循环 (ADC采集与处理) ---
  
  // 场景 1: 正在进行下线测试 (优先级最高)
  if (LineLow_GetState() == LINE_LOW_BUSY) 
  {
      // 启动一次 ADC 转换
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
          adc_value = HAL_ADC_GetValue(&hadc1);
          LED0_ON; // 测试期间亮灯
          
          // 将数据喂给 LineLow 模块处理
          // 如果处理完成，LineLow_Process 内部会打印结果并自动把状态切回 IDLE
          LineLow_Process(adc_value); 
      }
  }
  // 场景 2: 正在进行普通测量 (含延时停止期间)
  else if ((measure_state || (stop_flag && (HAL_GetTick() - stop_start_time) < STOP_DELAY)) && 
           HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) 
  {
      LED0_ON; // 测量期间亮灯
      
      // 1. 获取最新数据
      adc_value = HAL_ADC_GetValue(&hadc1);
      current_time = HAL_GetTick();
      
      // 2. 数据入队 (Ring Buffer 逻辑)
      if (buf_count < MAX_SAMPLES) {
          // 数组没满，直接加在后面
          buf_val[buf_count] = (float)adc_value;
          buf_time[buf_count] = current_time;
          buf_count++;
      } else {
          // 数组满了，挤掉最旧的一个 (移位)
          for(int i=0; i<MAX_SAMPLES-1; i++) {
              buf_val[i] = buf_val[i+1];
              buf_time[i] = buf_time[i+1];
          }
          buf_val[MAX_SAMPLES-1] = (float)adc_value;
          buf_time[MAX_SAMPLES-1] = current_time;
      }

      // 3. 清洗过期数据 (超过窗口时间 2000ms 的扔掉)
      while (buf_count > 0 && (current_time - buf_time[0] > WINDOW_MS)) {
          for (int i = 0; i < buf_count - 1; i++) {
              buf_val[i] = buf_val[i + 1];
              buf_time[i] = buf_time[i + 1];
          }
          buf_count--; 
      }

      // 4. 计算与打印
      uint32_t measure_duration = current_time - measure_start_time;
      
      if (measure_duration < WINDOW_MS) {
           // 前2秒预热期
           printf("[%"PRIu32" ms] Counting...\r\n", measure_duration);
      } else {
           // 稳定期，调用外部函数计算
           float val_sliding = Calc_Sliding_Average(buf_val, buf_count);
           float val_median = Calc_Median(buf_val, buf_count);
           
           printf("[%"PRIu32" ms] Sliding:%.2f Median:%.2f Raw:%"PRIu32"\r\n", 
                   measure_duration, val_sliding, val_median, adc_value);
      }

  }
  // 场景 3: 系统空闲
  else 
  {
      LED0_OFF; // 关灯
      
      // 如果普通测量完全结束了，清空缓存，防止下次启动时用到旧数据
      if (!measure_state && !stop_flag) {
          buf_count = 0; 
      }
  }
  
  HAL_Delay(47); // 采样间隔控制 (~50ms)
   
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
