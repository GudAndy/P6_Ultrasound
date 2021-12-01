/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "ultrasound.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MODE {
  SINGLE,
  CONT
} MODE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_SIZE 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t in_buffer[BUFF_SIZE], out_buffer[BUFF_SIZE];
uint8_t in_idx = 0;
uint8_t ch, st_ch;

uint8_t post_check = 0;
uint8_t begin_check = 0;
MODE mode = SINGLE;


uint32_t distance = 0;
uint8_t num_valid;
uint32_t min = UINT32_MAX;
uint32_t max = 0;
uint32_t data[100] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void POST();
void measure100(void);

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_UART_Receive_IT(&huart2, &ch, 1);
  HAL_TIM_Base_Start(&usTIM);

  uint8_t prompt = 1;
  while (1)
  {
    if (prompt){
      sprintf((char *) out_buffer, "Enter Command: ");
      HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
      prompt = 0;
    }

    HAL_UART_Receive_IT(&huart2, &ch, 1);
    if (ch == '\r'){
      prompt = 1;
      ch = '\0';
      if (strncmp((char *) in_buffer, "post", 4) == 0){
        POST();
      } else if (strncmp((char *) in_buffer, "sing", 4) == 0){
        distance = read_ultrasound();
        if (distance >= 50 && distance <= 1000){
          sprintf((char *) out_buffer, "Distance:\t%ld\n", distance);
          HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
        } else {
          sprintf((char *) out_buffer, "Distance:\t%s\n", "***");
          HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
        }
      } else if (strncmp((char *) in_buffer, "full", 4) == 0){
        measure100();
      }
      memset(in_buffer, '\0', BUFF_SIZE);
    }

    /* USER CODE END WHILE */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (ch != '\r' && ch != '\0'){
      in_buffer[in_idx] = ch;
      in_idx++;
      HAL_UART_Transmit_IT(huart, &ch, 1);
      HAL_UART_Receive_IT(huart, &ch, 1);
    } else if (ch == '\r') {
      in_buffer[in_idx] = '\r';
      memset(in_buffer + in_idx + 1, '\0', sizeof(in_buffer) - in_idx);  // Terminate everything after the return char
      in_idx = 0;
      HAL_UART_Transmit_IT(huart, &ch, 1);
    }
}

void POST(){
    HAL_UART_Transmit(&huart2, (uint8_t *) "\nPOSTING\n", 9, HAL_MAX_DELAY);
    distance = read_ultrasound();

    // Until success
    while (distance < 50 || distance > 1000){
      memset(out_buffer, '\0',  sizeof(out_buffer));
      sprintf((char *) out_buffer, "Object is outside of POST range!\n");
      HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);

      // Prompt retry
      memset(out_buffer, '\0',  sizeof(out_buffer));
      sprintf((char *) out_buffer, "Try Again? ");
      HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
      HAL_UART_Receive(&huart2, in_buffer, 1, HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, (uint8_t *) "\n", 1, HAL_MAX_DELAY);

      if (strncmp((char *) in_buffer, "n", 1) == 0){  // If don't want to retry
        distance = -1;
        memset(out_buffer, '\0',  sizeof(out_buffer));
        sprintf((char *) out_buffer, "POST Failed!\n");
        HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
        post_check = 2;
        break;
      }
      distance = read_ultrasound();
    }

  if (distance != -1){
    memset(out_buffer, '\0',  sizeof(out_buffer));
    sprintf((char *) out_buffer, "POST Passed!\n");
    HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
  }
}

void measure100(void){
  ch = '\0';
  for (int i = 0; i < 100; i++){ // Collect data
    if (ch != '\0'){
      break;
    }
    distance = read_ultrasound();
    if (distance < 50 || distance > 1000){  // If distance is out of range
      distance = 0;
    }
    data[i] = distance;
    HAL_Delay(100);
  }

  num_valid = 0;
  if (ch != '\0'){
    for (int i = 0; i < 100; i++){ // Get num_valid
      if (data[i] != 0){
        num_valid++;
      }
    }
    sprintf((char *) out_buffer, "\nNumber of Valid Measurements Before Stop:\t%d\n", num_valid);
    HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
  }
  HAL_UART_Transmit(&huart2, (uint8_t *) "{\n", 2, HAL_MAX_DELAY);
  for (int i = 0; i < 100; i++){
    if (data[i] < min && data[i] > 0){
      min = data[i];
    }
    if (data[i] > max){
      max = data[i];
    }
    if (data[i] != 0){
      sprintf((char *) out_buffer, "\t%d, %ld\n", i, data[i]);
      HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
    }
  }
  HAL_UART_Transmit(&huart2, (uint8_t *) "}\n", 2, HAL_MAX_DELAY);
  sprintf((char *) out_buffer, "Maximum Measured Distance:\t%ld\n", max);
  HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
  sprintf((char *) out_buffer, "Minimum Measured Distance:\t%ld\n", min);
  HAL_UART_Transmit(&huart2, out_buffer, strlen((char *) out_buffer), HAL_MAX_DELAY);
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
