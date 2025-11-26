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

/* ---------------------------------------------------------------------------
 * Firmware Version Information
 * ---------------------------------------------------------------------------
 *
 * Firmware Name : STM32 FreeRTOS CLI Playground
 * Board         : NUCLEO-F446RE
 * MCU           : STM32F446RE (Cortex-M4F)
 *
 * Version History:
 *   v1.0.0  - Basic LED timer using TIM2 (initial bring-up)
 *   v1.1.0  - Added UART2 support + interrupt-driven RX
 *   v1.2.0  - Added simple single-character CLI
 *   v1.3.0  - Added virtual temperature sensor module
 *   v1.4.0  - Added FreeRTOS tasks (LEDTask, SensorTask, CLITask)
 *   v1.5.0  - Added UART mutex for thread-safe logging
 *   v1.6.0  - Added multi-word CLI (help, status, sensor read, log start/stop)
 *   v1.7.0  - Added log queue to decouple logging from CLI input
 *   v1.8.0  - Added 'clear' / 'cls' command to clear terminal
 *
 * Upcoming:
 *   v1.9.0  - Real sensor over I2C (BME280)
 *   v2.0.0  - UART DMA circular buffer (idle line interrupt) + log levels
 *
 * Build Timestamp:
 *   __DATE__  - compile date
 *   __TIME__  - compile time
 *
 * ---------------------------------------------------------------------------
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Log message type: fixed-size string for log lines */
typedef struct
{
  char text[64];
} LogMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Firmware version macros */
#define FW_VERSION_MAJOR   1
#define FW_VERSION_MINOR   8
#define FW_VERSION_PATCH   0

#define FW_VERSION_STRING  "v1.8.0"
#define FW_BUILD_DATE      __DATE__
#define FW_BUILD_TIME      __TIME__

/* ANSI escape sequence to clear terminal screen and move cursor to home */
#define ANSI_CLEAR_SCREEN  "\033[2J\033[H"

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

/* Logging flag (controlled by CLI 'log start' / 'log stop' or 'l'/'x') */
volatile uint8_t logging_enabled = 0;

/* Task handles */
osThreadId_t ledTaskHandle;
osThreadId_t sensorTaskHandle;
osThreadId_t cliTaskHandle;

/* CLI input queue (ISR → CLI task) */
osMessageQueueId_t cliQueueHandle;

/* Log queue: SensorTask + timer ISR → CLITask */
osMessageQueueId_t logQueueHandle;

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

const osMessageQueueAttr_t logQueue_attributes = {
  .name = "logQueue"
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
 * @brief Legacy single-character CLI handler.
 * @note  Kept for reference; not used by CLITask anymore. New code uses
 *        handle_command_line() with multi-word commands.
 */
void handle_command(uint8_t c);

/**
 * @brief Multi-word CLI command handler.
 * @param line Null-terminated input line, e.g. "sensor read", "log start".
 */
void handle_command_line(const char *line);

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
  /* Start TIM2 base with interrupt (used for optional periodic logging) */
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

  /* Create log message queue (SensorTask / timer → CLITask) */
  logQueueHandle = osMessageQueueNew(16, sizeof(LogMessage_t), &logQueue_attributes);
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
  * @brief Period elapsed callback in non-blocking mode.
  * @note  Called from TIM2 interrupt. Here we only queue optional
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

      LogMessage_t logMsg;
      snprintf(logMsg.text, sizeof(logMsg.text), "LOG: %.1f C\r\n", temp);

      /* Best-effort enqueue: do NOT block in ISR */
      (void)osMessageQueuePut(logQueueHandle, &logMsg, 0, 0);
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
  * @brief Legacy single-character handler (no longer used by CLITask).
  * @note  Left here for reference. New CLI uses handle_command_line().
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
  * @brief Multi-word CLI parser.
  * @note  Supports both long forms and legacy short forms:
  *        - "help" or "h"
  *        - "status" or "s"
  *        - "version"
  *        - "clear" or "cls"
  *        - "led toggle" or "t"
  *        - "sensor read" or "r"
  *        - "log start" or "l"
  *        - "log stop" or "x"
  */
void handle_command_line(const char *line)
{
  /* Simple tokenizer: extract up to 3 words */
  char cmd[16]  = {0};
  char arg1[16] = {0};
  char arg2[16] = {0};

  /* Parse: cmd [arg1] [arg2] */
  (void)sscanf(line, "%15s %15s %15s", cmd, arg1, arg2);

  /* help / h */
  if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0)
  {
    uart_print(
      "\r\nCommands:\r\n"
      "  help                - show this help\r\n"
      "  status              - show system status\r\n"
      "  version             - show firmware version\r\n"
      "  clear / cls         - clear terminal screen\r\n"
      "  led toggle          - toggle LED\r\n"
      "  sensor read         - read sensor once\r\n"
      "  log start           - enable timer-based logging\r\n"
      "  log stop            - disable timer-based logging\r\n"
    );
    return;
  }

  /* status / s */
  if (strcmp(cmd, "s") == 0 || strcmp(cmd, "status") == 0)
  {
    uart_print("\r\nStatus: OK, timer+uart+RTOS running\r\n");
    return;
  }

  /* version */
  if (strcmp(cmd, "version") == 0)
  {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "\r\nFirmware %s\r\nBuilt on %s at %s\r\n",
             FW_VERSION_STRING, FW_BUILD_DATE, FW_BUILD_TIME);
    uart_print(buf);
    return;
  }

  /* clear / cls */
  if (strcmp(cmd, "clear") == 0 || strcmp(cmd, "cls") == 0)
  {
    uart_print(ANSI_CLEAR_SCREEN);
    uart_print("> ");
    return;
  }

  /* t (short form LED toggle) */
  if (strcmp(cmd, "t") == 0)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    uart_print("\r\nLED toggled\r\n");
    return;
  }

  /* led toggle */
  if (strcmp(cmd, "led") == 0)
  {
    if (strcmp(arg1, "toggle") == 0)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      uart_print("\r\nLED toggled\r\n");
    }
    else
    {
      uart_print("\r\nUsage: led toggle\r\n");
    }
    return;
  }

  /* r (short form sensor read) */
  if (strcmp(cmd, "r") == 0)
  {
    float temp_c = sensor_read_celsius();
    char msg[64];
    snprintf(msg, sizeof(msg), "\r\nSensor value: %.1f C\r\n", temp_c);
    uart_print(msg);
    return;
  }

  /* sensor read */
  if (strcmp(cmd, "sensor") == 0)
  {
    if (strcmp(arg1, "read") == 0)
    {
      float temp_c = sensor_read_celsius();
      char msg[64];
      snprintf(msg, sizeof(msg), "\r\nSensor value: %.1f C\r\n", temp_c);
      uart_print(msg);
    }
    else
    {
      uart_print("\r\nUsage: sensor read\r\n");
    }
    return;
  }

  /* log start / l */
  if (strcmp(cmd, "l") == 0 || (strcmp(cmd, "log") == 0 && strcmp(arg1, "start") == 0))
  {
    logging_enabled = 1;
    uart_print("\r\nLogging started\r\n");
    return;
  }

  /* log stop / x */
  if (strcmp(cmd, "x") == 0 || (strcmp(cmd, "log") == 0 && strcmp(arg1, "stop") == 0))
  {
    logging_enabled = 0;
    uart_print("\r\nLogging stopped\r\n");
    return;
  }

  /* If we reach here, command not recognized */
  uart_print("\r\nUnknown command. Type 'help' for a list of commands.\r\n");
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
  * @note  Periodically reads the virtual sensor and enqueues log messages.
  */
void SensorTask(void *argument)
{
  (void)argument;

  sensor_init();

  for (;;)
  {
    float temp = sensor_read_celsius();

    LogMessage_t logMsg;
    snprintf(logMsg.text, sizeof(logMsg.text), "RTOS LOG: %.1f C\r\n", temp);

    /* Non-blocking log enqueue; if queue is full, drop the log */
    (void)osMessageQueuePut(logQueueHandle, &logMsg, 0, 0);

    osDelay(1000);                          /* 1 s */
  }
}

/**
  * @brief CLI task
  * @note  Owns the UART for both logs and CLI output.
  *        - Drains logQueue and prints log messages.
  *        - Handles interactive CLI with line editing and prompt.
  */
void CLITask(void *argument)
{
  (void)argument;

  uint8_t c;
  char line_buf[64];
  uint16_t line_len = 0;

  LogMessage_t logMsg;

  uart_print("\r\nCLI Ready\r\n> ");

  for (;;)
  {
    /* ----------------------------------------------------------------------
     * 1) Drain any pending log messages first (non-blocking)
     * -------------------------------------------------------------------- */
    while (osMessageQueueGet(logQueueHandle, &logMsg, NULL, 0) == osOK)
    {
      /* Move to a fresh line, print the log, then redraw the prompt + input */
      uart_print("\r\n");
      uart_print(logMsg.text);

      uart_print("> ");
      if (line_len > 0)
      {
        /* Re-print whatever the user has typed so far */
        uart_print(line_buf);
      }
    }

    /* ----------------------------------------------------------------------
     * 2) Wait for next CLI character with a short timeout
     *    This lets us regularly check the log queue too.
     * -------------------------------------------------------------------- */
    if (osMessageQueueGet(cliQueueHandle, &c, NULL, 50) == osOK)
    {
      /* Handle newline (Enter) */
      if (c == '\r' || c == '\n')
      {
        uart_print("\r\n");

        if (line_len > 0)
        {
          line_buf[line_len] = '\0';   /* null-terminate */
          handle_command_line(line_buf);
          line_len = 0;
        }

        uart_print("> ");
        continue;
      }

      /* Handle backspace (ASCII 8 or 127) */
      if (c == 0x08 || c == 0x7F)
      {
        if (line_len > 0)
        {
          line_len--;
          /* Erase character on terminal: backspace, space, backspace */
          uart_print("\b \b");
        }
        continue;
      }

      /* Printable characters only, with bounds check */
      if (line_len < (sizeof(line_buf) - 1) && c >= 32 && c < 127)
      {
        line_buf[line_len++] = (char)c;
        uart_write_char(c);  /* echo */
      }
      /* else: ignore (buffer full or non-printable) */
    }
    /* else: timeout, loop again, process logs if any */
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
