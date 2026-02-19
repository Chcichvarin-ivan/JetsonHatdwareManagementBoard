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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "debug_console.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "btn_def.h"
#include "i2c_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
#define I2C_SLAVE_ADDRESS_7BIT   (0x12)  // pick a free address, Jetson uses this
#define I2C_TX_MAX_BYTES         (1 + 2*16)  // 1 count + up to 16 events (2 bytes each)
#define EVENT_RING_SIZE          64

#define BUTTON_COUNT             1

/* Debounce integrator parameters */
#define DEBOUNCE_MAX             4   // larger = more stable, slower response

/* Scan period */
#define BUTTON_SCAN_PERIOD_MS    5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
uint32_t ledTaskBuffer[ 128 ];
osStaticThreadDef_t ledTaskControlBlock;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .cb_mem = &ledTaskControlBlock,
  .cb_size = sizeof(ledTaskControlBlock),
  .stack_mem = &ledTaskBuffer[0],
  .stack_size = sizeof(ledTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
uint32_t buttonTaskBuffer[ 128 ];
osStaticThreadDef_t buttonTaskControlBlock;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .cb_mem = &buttonTaskControlBlock,
  .cb_size = sizeof(buttonTaskControlBlock),
  .stack_mem = &buttonTaskBuffer[0],
  .stack_size = sizeof(buttonTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
uint32_t debugTaskBuffer[ 128 ];
osStaticThreadDef_t debugTaskControlBlock;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .cb_mem = &debugTaskControlBlock,
  .cb_size = sizeof(debugTaskControlBlock),
  .stack_mem = &debugTaskBuffer[0],
  .stack_size = sizeof(debugTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for g_hostCmdQueue */
osMessageQueueId_t g_hostCmdQueueHandle;
uint8_t g_hostCmdQueueBuffer[ 16 * sizeof( uint8_t ) ];
osStaticMessageQDef_t g_hostCmdQueueControlBlock;
const osMessageQueueAttr_t g_hostCmdQueue_attributes = {
  .name = "g_hostCmdQueue",
  .cb_mem = &g_hostCmdQueueControlBlock,
  .cb_size = sizeof(g_hostCmdQueueControlBlock),
  .mq_mem = &g_hostCmdQueueBuffer,
  .mq_size = sizeof(g_hostCmdQueueBuffer)
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_RNG_Init(void);
void StartLedTask(void *argument);
void StartButtonTask(void *argument);
void StartDebugTask(void *argument);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of g_hostCmdQueue */
  g_hostCmdQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &g_hostCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* creation of debugTask */
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */


  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  I2C_SlaveApp_Init(&hi2c1);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B10E24;
  hi2c1.Init.OwnAddress1 = 128;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 94;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LED_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  DBG_OnUartRxCpltCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  DBG_OnUartTxCpltCallback(huart);
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
  // This callback fires when the counter reaches the EWI threshold,
  // which is inside the allowed refresh window.

    // Refresh back to 0x7F (or whatever counter you configured)
    HAL_WWDG_Refresh(hwwdg);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLedTask */
/* =========================
   Jetson -> Nucleo command protocol (I2C master write)
   ========================= */


typedef struct {
  uint8_t cmd;
  uint8_t value;
} HostCommand;
/**
  * @brief  Function implementing the ledTask thread.
  * @param  argument: Not used
  * @retval None
  */
static uint8_t LedState;
void setLedState(uint8_t in_state) {
  LedState = in_state;
}
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(pdMS_TO_TICKS(500));

    HAL_GPIO_TogglePin( LD3_GPIO_Port, LD3_Pin);
    if (LedState ==  I2C_SLAVE_LED_ON) {
      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
      /* USER CODE END RTOS_THREADS */
    }else if (LedState ==   I2C_SLAVE_LED_FLASH2H) {
      HAL_GPIO_TogglePin(GPIOB, LED_Pin);
    }else if (LedState == I2C_SLAVE_LED_OFF){
      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
    }else if (LedState == I2C_SLAVE_LED_SHTDWN) {
      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
    }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartButtonTask */


#define BTN_EVENT_Q_LEN  16

static StaticQueue_t g_btnQStruct;
static uint8_t g_btnQStorage[BTN_EVENT_Q_LEN * sizeof(btn_event_msg_t)];
static QueueHandle_t g_btnQ = NULL;

/* Create queue once at init (task context) */
static void BTN_EventQueueInit(void)
{
  if (g_btnQ) return;
  g_btnQ = xQueueCreateStatic(
      BTN_EVENT_Q_LEN,
      sizeof(btn_event_msg_t),
      g_btnQStorage,
      &g_btnQStruct
  );
}

/* Get handle so other tasks can receive events */
QueueHandle_t BTN_GetEventQueue(void)
{
  return g_btnQ;
}

/* Post event (task context). Non-blocking. */
static void BTN_PostEvent(btn_evt_type_t type, uint16_t pin, uint32_t now_ms)
{
  if (!g_btnQ) return;

  btn_event_msg_t e = {.type = type, .pin = pin, .t_ms = now_ms};
  (void)xQueueSend(g_btnQ, &e, 0); /* 0 tick wait: drop if full */
}
/* =========================
   Debounce state (integrator)
   ========================= */

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       active_low;
  /* ---- Debounce integrator ----
   * stable: debounced level (0=not pressed, 1=pressed)
   * integ:  integrator 0..db_max
   */
  uint8_t stable;     /* 0=not pressed, 1=pressed (debounced) */
  uint8_t integ;      /* 0..db_max */
  uint8_t db_max;     /* threshold: e.g. 4 for ~20ms at 5ms sampling */

  /* ---- Gesture timing ---- */
  uint32_t long_press_ms;      /* e.g. 800 */
  uint32_t double_window_ms;   /* e.g. 300 */
  uint32_t pressed_at_ms;      /* when stable became pressed */
  uint8_t  long_fired;

  /* ---- Double press tracking ---- */
  uint8_t  waiting_second;
  uint32_t first_release_ms;
} btn_t;

/* =========================
   Button pin mapping
   Update these to match your board wiring / CubeMX config.
   NOTE: NUCLEO-L432KC has a user button B1 on PA0 on some Nucleo boards,
         but L432KC Nucleo-32 variants vary. Use your actual pins.
   ========================= */
static btn_t g_db[BUTTON_COUNT] = {
  {GPIOA, GPIO_PIN_1,1,
      0, 0, DEBOUNCE_MAX, /* 5ms tick -> ~20ms debounce */
    1200,
    300,
    0,
    0,
    0,
    0,
  }
};


/* Read raw button (pressed=true/false) */
static inline uint8_t btn_read_raw_pressed(GPIO_TypeDef *port, uint16_t pin, uint8_t active_low)
{
  uint8_t level = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
  return active_low ? (uint8_t)(!level) : level; /* pressed=1 */
}

static uint8_t btn_debounce_update(btn_t *b, uint8_t raw_pressed)
{
  if (raw_pressed)
  {
    if (b->integ < b->db_max) b->integ++;
  }
  else
  {
    if (b->integ > 0) b->integ--;
  }

  if (b->integ == b->db_max && b->stable == 0) { b->stable = 1; return 1; }
  if (b->integ == 0 && b->stable == 1)         { b->stable = 0; return 1; }
  return 0;
}

void BTN_UpdateAndPost(btn_t *b, uint32_t now_ms)
{
  uint8_t raw = btn_read_raw_pressed(b->port, b->pin, b->active_low);
  uint8_t changed = btn_debounce_update(b, raw);

  /* Debounced press edge */
  if (changed && b->stable == 1)
  {
    b->pressed_at_ms = now_ms;
    b->long_fired = 0;
    /* do not post anything yet (short press is decided later) */
  }

  /* Long press detection while held */
  if (b->stable == 1 && !b->long_fired)
  {
    if ((now_ms - b->pressed_at_ms) >= b->long_press_ms)
    {
      b->long_fired = 1;
      b->waiting_second = 0; /* long cancels double */
      BTN_PostEvent(BTN_EVT_LONG_PRESS, b->pin, now_ms);
    }
  }

  /* Debounced release edge */
  if (changed && b->stable == 0)
  {
    if (b->long_fired)
    {
      /* already reported LONG; release produces no event */
      return;
    }

    if (b->waiting_second)
    {
      /* second click release within window => DOUBLE */
      if ((now_ms - b->first_release_ms) <= b->double_window_ms)
      {
        b->waiting_second = 0;
        BTN_PostEvent(BTN_EVT_DOUBLE_PRESS, b->pin, now_ms);
      }
      else
      {
        /* too late: treat as "new first" */
        b->first_release_ms = now_ms;
        b->waiting_second = 1;
      }
    }
    else
    {
      /* first short click released: start window */
      b->waiting_second = 1;
      b->first_release_ms = now_ms;
    }
  }

  /* Finalize single short press when window expires (no second press) */
  if (b->waiting_second && b->stable == 0)
  {
    if ((now_ms - b->first_release_ms) > b->double_window_ms)
    {
      b->waiting_second = 0;
      BTN_PostEvent(BTN_EVT_PRESSED, b->pin, now_ms);
    }
  }
}
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  (void)argument;

  BTN_EventQueueInit(); /* create queue once */
  /* Initialize debounce states */
  /* Infinite loop */
  uint32_t now_ms = 0;
  for(;;)
  {

    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
      now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
      BTN_UpdateAndPost(&g_db[i], now_ms);
    }
    osDelay(pdMS_TO_TICKS(BUTTON_SCAN_PERIOD_MS));
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the debugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */
  dbg_config_t cfg = {
    .huart = &huart2,
    .hi2c  = &hi2c1,
    .btn_port = GPIOC,
    .btn_pin  = GPIO_PIN_13,
    .baud_hint = 115200,
    .login_token = "k1-7a",
    .session_timeout_ms = 300000,
    .auto_session_on_login = 0,
    .default_echo = 1,
    .default_timestamps = 0,
    .default_prompt = 1,
  };

  DBG_Init(&cfg);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDebugTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
