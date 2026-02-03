/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 引脚对应关系（按下=低电平）:
  // PB6 (switch4) -> horizontal_min
  // PB7 (switch3) -> vertical_min
  // PB8 (switch1) -> vertical_max
  // PB9 (switch2) -> horizontal_max
  GPIO_PinState last_horizontal_min_state = GPIO_PIN_SET;
  GPIO_PinState last_vertical_min_state = GPIO_PIN_SET;
  GPIO_PinState last_vertical_max_state = GPIO_PIN_SET;
  GPIO_PinState last_horizontal_max_state = GPIO_PIN_SET;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 读取 4 个开关状态（按下=低电平）
    GPIO_PinState horizontal_min_state = HAL_GPIO_ReadPin(switch4_GPIO_Port, switch4_Pin); // PB6
    GPIO_PinState vertical_min_state = HAL_GPIO_ReadPin(switch3_GPIO_Port, switch3_Pin);   // PB7
    GPIO_PinState vertical_max_state = HAL_GPIO_ReadPin(switch1_GPIO_Port, switch1_Pin);   // PB8
    GPIO_PinState horizontal_max_state = HAL_GPIO_ReadPin(switch2_GPIO_Port, switch2_Pin); // PB9
    
    // 当任意一个开关为低电平时，PC13 拉低；否则拉高
    if (horizontal_min_state == GPIO_PIN_RESET ||
        vertical_min_state == GPIO_PIN_RESET ||
        vertical_max_state == GPIO_PIN_RESET ||
        horizontal_max_state == GPIO_PIN_RESET)
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
    
    // 处理 horizontal_min 的状态变化和持续输出
    if (horizontal_min_state != last_horizontal_min_state)
    {
      // 状态改变时发送相应消息
      if (horizontal_min_state == GPIO_PIN_RESET)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_min_1\r\n", 18, HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_min_0\r\n", 18, HAL_MAX_DELAY);
      }
      last_horizontal_min_state = horizontal_min_state;
    }
    else if (horizontal_min_state == GPIO_PIN_RESET)
    {
      // 保持低电平时，持续输出
      HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_min_1\r\n", 18, HAL_MAX_DELAY);
    }
    else if (horizontal_min_state == GPIO_PIN_SET)
    {
      // 保持高电平时，持续输出
      HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_min_0\r\n", 18, HAL_MAX_DELAY);
    }
    
    // 处理 vertical_min 的状态变化和持续输出
    if (vertical_min_state != last_vertical_min_state)
    {
      // 状态改变时发送相应消息
      if (vertical_min_state == GPIO_PIN_RESET)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_min_1\r\n", 16, HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_min_0\r\n", 16, HAL_MAX_DELAY);
      }
      last_vertical_min_state = vertical_min_state;
    }
    else if (vertical_min_state == GPIO_PIN_RESET)
    {
      // 保持低电平时，持续输出
      HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_min_1\r\n", 16, HAL_MAX_DELAY);
    }
    else if (vertical_min_state == GPIO_PIN_SET)
    {
      // 保持高电平时，持续输出
      HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_min_0\r\n", 16, HAL_MAX_DELAY);
    }

    // 处理 vertical_max 的状态变化和持续输出
    if (vertical_max_state != last_vertical_max_state)
    {
      if (vertical_max_state == GPIO_PIN_RESET)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_max_1\r\n", 16, HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_max_0\r\n", 16, HAL_MAX_DELAY);
      }
      last_vertical_max_state = vertical_max_state;
    }
    else if (vertical_max_state == GPIO_PIN_RESET)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_max_1\r\n", 16, HAL_MAX_DELAY);
    }
    else if (vertical_max_state == GPIO_PIN_SET)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"vertical_max_0\r\n", 16, HAL_MAX_DELAY);
    }

    // 处理 horizontal_max 的状态变化和持续输出
    if (horizontal_max_state != last_horizontal_max_state)
    {
      if (horizontal_max_state == GPIO_PIN_RESET)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_max_1\r\n", 18, HAL_MAX_DELAY);
      }
      else
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_max_0\r\n", 18, HAL_MAX_DELAY);
      }
      last_horizontal_max_state = horizontal_max_state;
    }
    else if (horizontal_max_state == GPIO_PIN_RESET)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_max_1\r\n", 18, HAL_MAX_DELAY);
    }
    else if (horizontal_max_state == GPIO_PIN_SET)
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"horizontal_max_0\r\n", 18, HAL_MAX_DELAY);
    }
    
    HAL_Delay(10);  // 短暂延时，避免CPU占用过高
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : switch4_Pin switch3_Pin switch1_Pin switch2_Pin */
  GPIO_InitStruct.Pin = switch4_Pin|switch3_Pin|switch1_Pin|switch2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
