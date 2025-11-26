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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "sensor.h"
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
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* -------------------------------------------------------------------------- */
/* Application RTOS objects                                                   */
/* -------------------------------------------------------------------------- */

/* Single-byte RX buffer for interrupt-driven UART */
uint8_t rx_byte;

/* Logging flag (controlled by CLI 'l' / 'x' commands, read in timer callback) */
volatile uint8_t logging_enabled = 0;

/* Task handles */
osThreadId_t ledTaskHandle;
osThreadId_t sensorTaskHandle;
osThreadId_t cliTaskHandle;

/* CLI input queue (ISR → CLI task) */
osMessageQueueId_t cliQueueHandle;

/* UART mutex for thread-safe printing from multiple tasks */
osMutexId_t uartMutexHandle;

/* Task attributes */
const osThreadAttr_t ledTask_attributes = {
  .name       = "ledTask",
  .priority   = osPriorityNormal,
  .stack_size = 128 * 4
};

const osThreadAttr_t sensorTask_attributes = {
  .name       = "sensorTask",
  .priority   = osPriorityBelowNormal,
  .stack_size = 256 * 4
};

const osThreadAttr_t cliTask_attributes = {
  .name       = "cliTask",
  .priority   = osPriorityAboveNormal,
  .stack_size = 512 * 4
};

/* Message queue attributes */
const osMessageQueueAttr_t cliQueue_attributes = {
  .name = "cliQueue"
};

/* UART mutex attributes */
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* -------------------------------------------------------------------------- */
/* Private function prototypes (user code)                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Thread-safe UART print helper (null-terminated strings).
 */
void uart_print(const char *msg);

/**
 * @brief Single-character UART TX helper (uses same mutex as uart_print).
 */
void uart_write_char(uint8_t c);

/**
 * @brief Handle one CLI command character.
 */
void handle_command(uint8_t c);

/* RTOS task entry functions */
void LEDTask(void *argument);
void SensorTask(void *argument);
void CLITask(void *argument);
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
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  /* Start TIM2 base with interrupt (used for optional logging) */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Create UART mutex for thread-safe printing */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Create CLI message queue (ISR → CLI task) */
  cliQueueHandle = osMessageQueueNew(32, sizeof(uint8_t), &cliQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  /* Create application tasks */
  ledTaskHandle    = osThreadNew(LEDTask,    NULL, &ledTask_attributes);
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  cliTaskHandle    = osThreadNew(CLITask,    NULL, &cliTask_attributes);

  uart_print("System boot OK\r\n");

  /* Start UART RX interrupt for 1-byte CLI input */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef   sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef  sMasterConfig      = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance           = TIM2;
  htim2.Init.Prescaler     = 16000 - 1;
  htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim2.Init.Period        = 500 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
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

  huart2.Instance        = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Period elapsed callback in non-blocking mode
  * @note  Called from TIM2 interrupt. Here we only handle optional
  *        periodic logging (if logging_enabled == 1). LED blinking
  *        is handled in LEDTask instead.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (logging_enabled)
    {
      float temp = sensor_read_celsius();
      char msg[64];
      snprintf(msg, sizeof(msg), "LOG: %.1f C\r\n", temp);
      uart_print(msg);
    }
  }
}

/**
  * @brief Thread-safe UART string transmit helper.
  */
void uart_print(const char *msg)
{
  if ((osKernelGetState() == osKernelRunning) && (uartMutexHandle != NULL))
  {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(uartMutexHandle);
  }
  else
  {
    /* Before RTOS is running, print without mutex */
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
}

/**
  * @brief Thread-safe UART single-character transmit helper.
  */
void uart_write_char(uint8_t c)
{
  if ((osKernelGetState() == osKernelRunning) && (uartMutexHandle != NULL))
  {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
    osMutexRelease(uartMutexHandle);
  }
  else
  {
    HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
  }
}

/**
  * @brief UART RX complete callback.
  * @note  Called in ISR context. Only pushes the received byte into
  *        the CLI queue and re-arms the UART.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Push received byte into CLI queue (non-blocking) */
    osMessageQueuePut(cliQueueHandle, &rx_byte, 0, 0);

    /* Re-arm RX for next byte */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

/**
  * @brief Handle one CLI command character.
  */
void handle_command(uint8_t c)
{
  switch (c)
  {
    case 'h':
      uart_print("\r\nCommands: h=help, t=toggle LED, s=status, r=read, l=start log, x=stop log\r\n");
      break;

    case 't':
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      uart_print("\r\nLED toggled\r\n");
      break;

    case 'r':
    {
      float temp_c = sensor_read_celsius();
      char msg[64];
      snprintf(msg, sizeof(msg), "\r\nSensor value: %.1f C\r\n", temp_c);
      uart_print(msg);
      break;
    }

    case 'l':
      logging_enabled = 1;
      uart_print("\r\nLogging started\r\n");
      break;

    case 'x':
      logging_enabled = 0;
      uart_print("\r\nLogging stopped\r\n");
      break;

    case 's':
      uart_print("\r\nStatus: OK, timer+uart+RTOS running\r\n");
      break;

    default:
      uart_print("\r\nUnknown cmd. Use h for help.\r\n");
      break;
  }
}

/**
  * @brief LED task
  * @note  Blinks LD2 every 500 ms as a heartbeat.
  */
void LEDTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  /* LD2 */
    osDelay(500);                           /* 500 ms */
  }
}

/**
  * @brief Sensor task
  * @note  Periodically reads the virtual sensor and logs over UART.
  */
void SensorTask(void *argument)
{
  (void)argument;

  sensor_init();

  for (;;)
  {
    float temp = sensor_read_celsius();
    char msg[64];
    snprintf(msg, sizeof(msg), "RTOS LOG: %.1f C\r\n", temp);
    uart_print(msg);
    osDelay(1000);                          /* 1 s */
  }
}

/**
  * @brief CLI task
  * @note  Waits for characters from the CLI queue, echoes them,
  *        and dispatches commands to handle_command().
  */
void CLITask(void *argument)
{
  (void)argument;

  uint8_t c;

  uart_print("\r\nCLI Ready\r\n");

  for (;;)
  {
    /* Wait indefinitely for next character from ISR → queue */
    if (osMessageQueueGet(cliQueueHandle, &c, NULL, osWaitForever) == osOK)
    {
      /* Echo back */
      uart_write_char(c);

      /* Process as a command */
      handle_command(c);
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END 5 */
//}

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
