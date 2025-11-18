/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Task 2 – Print Accelerometer + Gyroscope Values (Integer)
  *                   (LSM303AGR via I2C1 + I3G4250D via SPI1)
  *                   STM32F3-Discovery Board
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                            Sensor Definitions                              */
/* -------------------------------------------------------------------------- */
#define LSM_ACC_ADDR_READ   (0x33)     // 0x33 = read, 0x32 = write
#define OUT_X_L_A           0x28U
#define I3G4250D_CTRL_REG1  0x20U
#define I3G4250D_OUT_X_L    0x28U
#define I3G4250D_OUT_X_H    0x29U
#define I3G4250D_OUT_Y_L    0x2A
#define I3G4250D_OUT_Y_H    0x2B
#define LSM_ACC_SCALE_G     0.0039f    // 3.9 mg/LSB -> g
#define GYRO_SCALE_DPS      0.00875f   // 8.75 mdps/LSB -> °/s

#define GYRO_CS_PORT        CS_I2C_SPI_GPIO_Port
#define GYRO_CS_PIN         CS_I2C_SPI_Pin

/* -------------------------------------------------------------------------- */
/*                             Global Variables                               */
/* -------------------------------------------------------------------------- */
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
static char uart_buf[128];

/* -------------------------------------------------------------------------- */
/*                          Function Declarations                             */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

/* Sensor helpers */
static void LSM_Init(void);
static void GYRO_Init(void);
static int16_t LSM_Read_AccX_raw(void);
static int16_t GYRO_Read_GX_raw(void);
static uint8_t gyro_read_u8(uint8_t reg);
static void gyro_write_reg(uint8_t reg, uint8_t val);

/* -------------------------------------------------------------------------- */
/*                                   MAIN                                     */
/* -------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Chip Select high (idle) */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

  /* Initialize sensors */
  LSM_Init();
  GYRO_Init();

  /* Print header */
  int header = snprintf(uart_buf, sizeof(uart_buf), "ACC_X(g_int), GYRO_X(dps_int)\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)header, HAL_MAX_DELAY);

  /* Main Loop */
  while (1)
  {
    /* Read accel X raw and convert to integer g */
    int16_t ax_raw = LSM_Read_AccX_raw();
    int ax_int = (int)((float)ax_raw * LSM_ACC_SCALE_G);  // rounded integer g

    /* Read gyro X raw and convert to integer dps */
    int16_t gx_raw = GYRO_Read_GX_raw();
    int gx_int = (int)((float)gx_raw * GYRO_SCALE_DPS);   // rounded integer °/s

    /* Print integer values */
    int n = snprintf(uart_buf, sizeof(uart_buf), "%d, %d\r\n", ax_int, gx_int);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, HAL_MAX_DELAY);

    HAL_Delay(100);
  }
}

/* -------------------------------------------------------------------------- */
/*                          Sensor Functions                                  */
/* -------------------------------------------------------------------------- */
static void LSM_Init(void)
{
  uint8_t ctrl1 = 0x67;  // Normal mode, all axes enabled
  uint8_t ctrl4 = 0x00;  // ±2g
  uint8_t dev = (LSM_ACC_ADDR_READ & 0xFE); // 0x32 write

  HAL_I2C_Mem_Write(&hi2c1, dev, 0x20, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, HAL_MAX_DELAY);
  HAL_Delay(5);
  HAL_I2C_Mem_Write(&hi2c1, dev, 0x23, I2C_MEMADD_SIZE_8BIT, &ctrl4, 1, HAL_MAX_DELAY);
  HAL_Delay(10);
}

static void GYRO_Init(void)
{
  HAL_Delay(10);
  gyro_write_reg(I3G4250D_CTRL_REG1,0xBF );  // Power on, enable XYZ 0b1011 1111
  HAL_Delay(10);
}

static int16_t LSM_Read_AccX_raw(void)
{
  uint8_t buf[6];
  uint8_t addr = OUT_X_L_A | 0x80U; // auto-increment
  HAL_I2C_Mem_Read(&hi2c1, LSM_ACC_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
  return (int16_t)((buf[1] << 8) | buf[0]);
}

static int16_t GYRO_Read_GX_raw(void)
{
  uint8_t xl = gyro_read_u8(I3G4250D_OUT_Y_L);
  uint8_t xh = gyro_read_u8(I3G4250D_OUT_Y_H);
  return (int16_t)((xh << 8) | xl);
}

static uint8_t gyro_read_u8(uint8_t reg)
{
  uint8_t cmd = reg | 0x80U;
  uint8_t val = 0;
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &val, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  return val;
}

static void gyro_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { reg & 0x7F, val };
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/*                        Peripheral Init Functions                           */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;  // must be 8-bit
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, GYRO_CS_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GYRO_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GYRO_CS_PORT, &GPIO_InitStruct);
}

/* -------------------------------------------------------------------------- */
/*                              Error Handler                                 */
/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* optional: print or log error */
}
#endif
