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
I2C_HandleTypeDef hi2c1;
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ====== CONFIG ======
#define SEG_PORT_A GPIOA            // b..g, dp on GPIOA
#define SEG_PORT_B GPIOB            // a on PB0

// --- Professor's ACTIVE-LOW LUT (0 = ON, 1 = OFF), bit0=a ... bit6=g ---
static const uint8_t HEX_TO_SEG[16] = {
  0x40, // 0
  0x79, // 1
  0x24, // 2
  0x30, // 3
  0x19, // 4
  0x12, // 5
  0x02, // 6
  0x78, // 7
  0x00, // 8
  0x10, // 9
  0x08, // A
  0x03, // b
  0x46, // C
  0x21, // d
  0x06, // E
  0x0E  // F
};

// --- Task 2: Student ID digits (CHANGE THESE TO YOUR ID) ---
static const uint8_t STUDENT_ID[] = { 9,2,0,1 }; // <-- replace with your actual ID digits
#define ID_LEN  (sizeof(STUDENT_ID)/sizeof(STUDENT_ID[0]))

// On most STM32 boards with PA0 user button: pressed = HIGH. Invert if needed.
static inline uint8_t user_button_pressed(void) {
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
}

// Drive segments given 7-bit mask (a..g) and dp flag (active-low logic)
static inline void seg_write_raw(uint8_t segs_7bit, uint8_t dp_on)
{
  // ACTIVE-LOW: 0=ON, 1=OFF ; bit0=a ... bit6=g ; bit7=dp
  uint8_t out = (segs_7bit & 0x7F) | ((dp_on ? 0U : 1U) << 7);

  // a on PB0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (out & (1<<0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // b..g, dp on GPIOA
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (out & (1<<1)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // b
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (out & (1<<2)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // c
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (out & (1<<3)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // d
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (out & (1<<4)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // e
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (out & (1<<5)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // f
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (out & (1<<6)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // g
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (out & (1<<7)) ? GPIO_PIN_SET : GPIO_PIN_RESET); // dp
}


static inline void display_digit(uint8_t d) {
  seg_write_raw(HEX_TO_SEG[d % 16], 0 /* dp off */);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  // --- Task 2: show first digit, then advance on each button press (debounced) ---
  uint8_t idx = 0;
  display_digit(STUDENT_ID[idx]);

  uint8_t last_read = 0;            // last raw sample
  uint8_t btn_state = 0;            // debounced stable state
  uint32_t last_change_ms = 0;
  const uint32_t DEBOUNCE_MS = 30;  // 20–40ms typical
  
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint8_t reading = user_button_pressed();
    uint32_t now = HAL_GetTick();

    if (reading != last_read) {
      last_change_ms = now; // input toggled: restart debounce timer
    }

    if ((now - last_change_ms) > DEBOUNCE_MS) {
      if (reading != btn_state) {
        btn_state = reading;
        if (btn_state) { // rising edge (pressed)
          idx = (idx + 1) % ID_LEN;   // next digit, wrap to start
          display_digit(STUDENT_ID[idx]);
        }
      }
    }

    last_read = reading;
    HAL_Delay(1);
    /* USER CODE END 3 */
  }
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USB Initialization Function
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* GPIOE (unchanged) */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // ==== Our seven-seg pins ====

  // Initial levels: OFF at boot (ACTIVE-LOW → drive HIGH)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                            GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // 'a'

  // PA0 as USER BUTTON input
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // use GPIO_PULLDOWN if floating
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1..PA7 as outputs for b..g,dp
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
                        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB0 as output for 'a'
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
