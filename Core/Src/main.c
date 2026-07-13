/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * NOTE: Integrated with the actuation-circuit firmware (App/ modules).
  *   - i2c_app.c/.h REMOVED (replaced by App/ i2c_reg.c + app.c).
  *   - ledTask now drives the RGB status indicator (App_LedTick).
  *   - buttonTask gesture engine kept as-is; it just reports liveness.
  *   - App_Init() replaces I2C_SlaveApp_Init() and creates the FSM/telemetry
  *     tasks. Pass &htim2 instead of NULL once TIM2 is added in CubeMX.
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
#include "app.h"          /* was: i2c_app.h */
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
#define BUTTON_COUNT             1
#define DEBOUNCE_MAX             4
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

TIM_HandleTypeDef htim2;   /* actuation: Ch.1/Ch.2 PWM + Ch.4 capture */

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
static void MX_TIM2_Init(void);
void StartLedTask(void *argument);
void StartButtonTask(void *argument);
void StartDebugTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  g_hostCmdQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &g_hostCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Actuation-circuit firmware bring-up (TIM2 configured above). */
  App_Init(&hi2c1, &htim2);
  /* USER CODE END RTOS_EVENTS */

  osKernelStart();

  while (1)
  {
    /* USER CODE BEGIN WHILE */
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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

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

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LED_Pin, GPIO_PIN_RESET);

  /* PA5 (A4): user button, active-low + pull-up.
   * (SB16/SB18 removed on this board, so PA5 works as a normal GPIO.) */
  GPIO_InitStruct.Pin = BTN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA4: Ch.3 external actuation sensor - EXTI rising edge */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA0/PA1/PA3 -> TIM2 CH1/CH2/CH4 (AF1); configured in HAL_TIM_Base_MspInit */

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /*Configure GPIO pins : LD3_Pin LED_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* RGB status LEDs (App/rgb_led.c) self-configure their own pins in
   * rgb_led_init(); nothing needed here unless you want them pre-driven. */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ------- Actuation timer TIM2: 1 us tick, 50 Hz; CH1/CH2 PWM, CH4 capture -------
 * Hand-authored (regeneration-note in INTEGRATION.md). Enables clock, GPIO AF
 * (PA0/PA1/PA3 = AF1_TIM2) and the TIM2 NVIC line in HAL_TIM_Base_MspInit. */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA0=CH1 (out), PA1=CH2 (out), PA3=CH4 (capture in) */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM2_PSC;                 /* 79 -> 1 MHz -> 1 us */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_ARR;                    /* 19999 -> 20 ms / 50 Hz */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = US_STANDBY;                    /* initial 1150 us */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  /* PWM start / capture start are issued by pwm_out_init() and telemetry_init(). */
}

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
  /* Refresh ONLY while every required task is alive. A hung task stops the
   * refresh and the WWDG resets the MCU. */
  if (App_WwdgShouldRefresh())
  {
    HAL_WWDG_Refresh(hwwdg);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  ledTask now drives the RGB status indicator via App_LedTick().
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;
  for(;;)
  {
    App_LedTick();
    osDelay(pdMS_TO_TICKS(RGBLED_PERIOD_MS));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartButtonTask */

#define BTN_EVENT_Q_LEN  16

static StaticQueue_t g_btnQStruct;
static uint8_t g_btnQStorage[BTN_EVENT_Q_LEN * sizeof(btn_event_msg_t)];
static QueueHandle_t g_btnQ = NULL;

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

QueueHandle_t BTN_GetEventQueue(void)
{
  return g_btnQ;
}

static void BTN_PostEvent(btn_evt_type_t type, uint16_t pin, uint32_t now_ms)
{
  btn_event_msg_t e = {.type = type, .pin = pin, .t_ms = now_ms};
  /* 1) debug console `btn session` reads this queue */
  if (g_btnQ) (void)xQueueSend(g_btnQ, &e, 0);
  /* 2) actuation FSM gets its own copy (safety actions never stolen) */
  App_PostButtonEvent(&e);
}

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       active_low;
  uint8_t stable;
  uint8_t integ;
  uint8_t db_max;
  uint32_t long_press_ms;
  uint32_t double_window_ms;
  uint32_t pressed_at_ms;
  uint8_t  long_fired;
  uint8_t  waiting_second;
  uint32_t first_release_ms;
} btn_t;

static btn_t g_db[BUTTON_COUNT] = {
  {BTN_GPIO_PORT, BTN_GPIO_PIN, 1,   /* button PA5/A4 (SB16/SB18 removed) */
    0, 0, DEBOUNCE_MAX,
    1200,
    300,
    0, 0, 0, 0,
  }
};

static inline uint8_t btn_read_raw_pressed(GPIO_TypeDef *port, uint16_t pin, uint8_t active_low)
{
  uint8_t level = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
  return active_low ? (uint8_t)(!level) : level;
}

static uint8_t btn_debounce_update(btn_t *b, uint8_t raw_pressed)
{
  if (raw_pressed) { if (b->integ < b->db_max) b->integ++; }
  else             { if (b->integ > 0)         b->integ--; }
  if (b->integ == b->db_max && b->stable == 0) { b->stable = 1; return 1; }
  if (b->integ == 0 && b->stable == 1)         { b->stable = 0; return 1; }
  return 0;
}

void BTN_UpdateAndPost(btn_t *b, uint32_t now_ms)
{
  uint8_t raw = btn_read_raw_pressed(b->port, b->pin, b->active_low);
  uint8_t changed = btn_debounce_update(b, raw);

  if (changed && b->stable == 1)
  {
    b->pressed_at_ms = now_ms;
    b->long_fired = 0;
  }

  if (b->stable == 1 && !b->long_fired)
  {
    if ((now_ms - b->pressed_at_ms) >= b->long_press_ms)
    {
      b->long_fired = 1;
      b->waiting_second = 0;
      BTN_PostEvent(BTN_EVT_LONG_PRESS, b->pin, now_ms);
    }
  }

  if (changed && b->stable == 0)
  {
    if (b->long_fired) { return; }

    if (b->waiting_second)
    {
      if ((now_ms - b->first_release_ms) <= b->double_window_ms)
      {
        b->waiting_second = 0;
        BTN_PostEvent(BTN_EVT_DOUBLE_PRESS, b->pin, now_ms);
      }
      else
      {
        b->first_release_ms = now_ms;
        b->waiting_second = 1;
      }
    }
    else
    {
      b->waiting_second = 1;
      b->first_release_ms = now_ms;
    }
  }

  if (b->waiting_second && b->stable == 0)
  {
    if ((now_ms - b->first_release_ms) > b->double_window_ms)
    {
      b->waiting_second = 0;
      BTN_PostEvent(BTN_EVT_PRESSED, b->pin, now_ms);
    }
  }
}
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  (void)argument;

  BTN_EventQueueInit();
  uint32_t now_ms = 0;
  for(;;)
  {
    App_NoteButtonAlive();   /* watchdog liveness */
    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
      now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
      BTN_UpdateAndPost(&g_db[i], now_ms);
    }
    osDelay(pdMS_TO_TICKS(BUTTON_SCAN_PERIOD_MS));
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDebugTask */
}

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

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
