/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Frequency measurement using TIM3 Input Capture interrupts
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Variables for Input Capture frequency measurement
volatile uint32_t last_capture = 0;
volatile uint32_t period = 0;
volatile uint32_t frequency = 0;  // Frequency in Hz
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN 0 */

/**
  * @brief  Input Capture callback called by HAL_TIM_IRQHandler on capture event
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if(current_capture >= last_capture)
    {
      period = current_capture - last_capture;
    }
    else
    {
      // Timer overflow occurred
      period = (htim->Init.Period - last_capture) + current_capture + 1;
    }
    last_capture = current_capture;

    if(period != 0)
    {
      // Get timer clock frequency (APB1 timer clocks run at 2x PCLK1 if prescaler > 1)
      uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;

      // Calculate frequency (Hz)
      frequency = timer_clock / period;
    }
  }
}

/**
  * @brief TIM3 IRQ Handler
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();

  // Enable TIM3 interrupt in NVIC
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  // Start TIM3 Input Capture interrupt mode (Channel 1)
  if(HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  while (1)
  {
    // frequency contains the measured input signal frequency in Hz
    // Use it here (send via UART, display, etc.)
    HAL_Delay(100);
  }
}

/**
  * @brief System Clock Configuration
  * (Adjust according to your clock setup, here is a sample for STM32F1)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Initialize HSE and PLL (8 MHz external crystal example)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8 MHz * 9 = 72 MHz SYSCLK
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  // Select PLL as system clock source
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;    // HCLK = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     // PCLK1 = 36 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     // PCLK2 = 72 MHz
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/**
  * @brief TIM3 Initialization Function
  */
static void MX_TIM3_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;             // No prescaler, max timer resolution
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;           // 16-bit max period
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_IC_Init(&htim3) != HAL_OK) Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;  // Capture rising edge
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;        // Direct input
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;                  // No prescaler on input
  sConfigIC.ICFilter = 0;                                   // No filter

  if(HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  // Enable GPIO clocks and configure TIM3 CH1 pin as alternate function input
  // For STM32F103, TIM3_CH1 is on PA6

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Input capture uses input mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;      // No pull-up/down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief Stub for I2C Init (dummy, remove if unused)
  */
static void MX_I2C1_Init(void)
{
  // Empty or implement if you use I2C
}

/**
  * @brief Stub for SPI Init (dummy, remove if unused)
  */
static void MX_SPI1_Init(void)
{
  // Empty or implement if you use SPI
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while(1)
  {
    // Stay here on error
  }
}
