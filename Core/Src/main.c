/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "RC.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_RX_BUF_SIZE 18
#define RC_RX_DATA_SIZE 18

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define KEY_IS_PRESSED(key) ((rx_data_[6] & (1 << (key))) != 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct {
  float r_row; // 右摇杆纵向
  float r_col; // 右摇杆横向
  float l_row; // 左摇杆纵向
  float l_col; // 左摇杆横向
  float mouse_x; // 鼠标 X 轴
  float mouse_y; // 鼠标 Y 轴
  float mouse_z; // 鼠标 Z 轴
} RCChannel;

// 定义开关状态的枚举类型
typedef enum {
  SWITCH_OFF = 0, // 关闭状态
  SWITCH_ON = 1   // 打开状态
} SwitchState;

// 定义鼠标按钮的枚举类型
typedef enum {
  MOUSE_BUTTON_RELEASED = 0, // 按钮未按下
  MOUSE_BUTTON_PRESSED = 1    // 按钮已按下
} MouseButtonState;

// 定义结构体来存储开关状态和鼠标按钮
typedef struct {
  SwitchState l; // 左开关状态
  SwitchState r; // 右开关状态
  MouseButtonState left_button; // 左鼠标按钮状态
  MouseButtonState right_button; // 右鼠标按钮状态
} RCSwitch;

// 定义全局变量
volatile uint8_t data_ready = 0;
uint8_t rx_buf_[RC_RX_BUF_SIZE];
uint8_t rx_data_[RC_RX_DATA_SIZE];
RCChannel channel_;
RCSwitch switch_;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_data()
{
  // 假设数据格式为：[r_row, r_col, l_row, l_col, switch_l, switch_r, mouse_x, mouse_y, left_button, right_button]
  channel_.r_row = (float)rx_data_[0] / 127.5f - 1.0f; // 映射到[-1, 1]
  channel_.r_col = (float)rx_data_[1] / 127.5f - 1.0f;
  channel_.l_row = (float)rx_data_[2] / 127.5f - 1.0f;
  channel_.l_col = (float)rx_data_[3] / 127.5f - 1.0f;

  // 鼠标坐标映射到[-1, 1]
  channel_.mouse_x = (float)(rx_data_[6] - 127.5f) / 127.5f; // 假设鼠标数据范围为0-255
  channel_.mouse_y = (float)(rx_data_[7] - 127.5f) / 127.5f;

  // 解析开关状态并将其转换为枚举类型
  switch_.l = (SwitchState)rx_data_[4]; // 解析左开关状态
  switch_.r = (SwitchState)rx_data_[5]; // 解析右开关状态

  // 解析鼠标按钮状态
  switch_.left_button = (MouseButtonState)rx_data_[8]; // 解析左鼠标按钮状态
  switch_.right_button = (MouseButtonState)rx_data_[9]; // 解析右鼠标按钮状态

  // 处理按键输入状态
  if (KEY_IS_PRESSED(0)) {
    // 处理按键0按下的逻辑
  }
  if (KEY_IS_PRESSED(1)) {
    // 处理按键1按下的逻辑
  }

  // 在这里可以处理或打印通道、开关和鼠标的状态
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    if(data_ready)
    {


      data_ready = 0;
    }
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, rx_buf_, RC_RX_BUF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if(data_ready)
    {
      process_data(); //process and analysis received data
      data_ready = 0; //reset
    }
    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
