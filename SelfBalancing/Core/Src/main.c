/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h> // Required for atan2 and fabs
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM303_ACC_ADDR      0x32   // I2C Address (0x19 << 1)
#define LSM303_CTRL_REG1_A   0x20
#define LSM303_OUT_X_L_A     0x28

// L3GD20 (Gyroscope)
#define L3GD20_CTRL_REG1     0x20
#define L3GD20_OUT_X_L       0x28
#define GYRO_CS_PIN          GPIO_PIN_3  // PE3 is Chip Select
#define GYRO_CS_PORT         GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define MPU6050_ADDR 0xD0
int16_t Accel_Y_RAW, Accel_Z_RAW;
float Current_Angle = 0.0;
char UART_Buffer[60]; // Buffer for text
// Add these for the Gyro and Filter
int16_t Gyro_X_RAW;
float Gyro_Rate_X = 0.0;
float dt = 0.01;            // Loop time (approx 10ms)
float Accel_Angle = 0.0;

int16_t Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW;

// --- PID VARIABLES ---
float Kp = 35.0;  // Proportional Gain (Tune this!)
float Ki = 0.8;   // Integral Gain
float Kd = 1.2;   // Derivative Gain
float PID_Total = 0.0;
float Error = 0.0;
float Previous_Error = 0.0;
float Integral = 0.0;
float Derivative = 0.0;

// --- MOTOR & ENCODER ---
volatile int32_t Encoder_L_Count = 0;
volatile int32_t Encoder_R_Count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t constrain_val(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// void Motor_Control(int16_t speed_L, int16_t speed_R) {
//     // --- LEFT MOTOR ---
//     // PWM: PC6 (TIM8_CH1) -> D11
//     // DIR: PC9 (D12) & PC8 (D10)
    
//     if (speed_L > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // PC9 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // PC8 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // PC9 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // PC8 High
//     }
//     // Drive PC6 (TIM8_CH1)
//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, abs(speed_L));

//     // --- RIGHT MOTOR ---
//     // PWM: PC7 (TIM8_CH2) -> D9
//     // DIR: PC10 (D8) & PC11 (D7)
    
//     if (speed_R > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // PC10 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // PC10 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // PC11 High
//     }
//     // Drive PC7 (TIM8_CH2)
//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs(speed_R));
// }


// void Motor_Control(int16_t speed_L, int16_t speed_R) {
//     // --- RIGHT MOTOR (PWM: D10/PC8) ---
//     // Forward: D8 (PC10) High, D12 (PC9) Low
//     if (speed_R > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // D8 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // D12 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // D8 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    // D12 High
//     }
//     // Drive PC8 (TIM8 Channel 3)
//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, abs(speed_R));

//     // --- LEFT MOTOR (PWM: D9/PC7) ---
//     // Forward: D7 (PC11) High, D6 (PC6) Low
//     if (speed_L > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // D7 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // D6 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // D7 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    // D6 High
//     }
//     // Drive PC7 (TIM8 Channel 2)
//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs(speed_L));
// }

// void Motor_Control(int16_t speed_L, int16_t speed_R) {
//     // --- RIGHT MOTOR (PWM: D10/PC8) ---
//     // Forward: D8 (PC10) High, D12 (PC9) Low
//     if (speed_R > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // D8 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // D12 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // D8 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    // D12 High
//     }
//     // Drive PC8 (TIM8 Channel 3)
//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs(speed_R));

//     // --- LEFT MOTOR (PWM: D9/PC7) ---
//     // Forward: D7 (PC11) High, D6 (PC6) Low
//     if (speed_L > 0) {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // D8 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // D12 Low

//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // D7 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  // D6 Low
//     } else {
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // D8 High
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // D12 Low


//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // D7 Low
//         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    // D6 High
//     }
//     // Drive PC7 (TIM8 Channel 2)
//       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs(speed_L));

//     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, abs(speed_L));
// }

void Motor_Control(int16_t speed_L, int16_t speed_R) {
    // --- LEFT MOTOR (PWM: PC6 - TIM8 CH1) ---
    // Direction Pins: PC9 (D12) and PC8 (D10)
    if (speed_L > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // Forward
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Backward (Swapped)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // Backward (Swapped)
    }
    // Use the argument 'speed_L', NOT the global 'speed'
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, abs(speed_L));

    // --- RIGHT MOTOR (PWM: PC7 - TIM8 CH2) ---
    // Direction Pins: PC10 (D8) and PC11 (D7)
    if (speed_R > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   // Forward
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Backward (Swapped)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // Backward (Swapped)
    }
    // Use the argument 'speed_R', NOT the global 'speed'
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, abs(speed_R));
}



/* USER CODE END 0 */

// Encoder Interrupts (PD0/PD2) - Remains the same
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // Left Encoder
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET) Encoder_L_Count++;
        else Encoder_L_Count--;
    }
    if (GPIO_Pin == GPIO_PIN_2) { // Right Encoder
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET) Encoder_R_Count--;
        else Encoder_R_Count++;
    }
}
/* USER CODE END 0 */

/* USER CODE BEGIN 0 */
void Init_Sensors(void) {
    // 1. Initialize Accelerometer (LSM303) via I2C
    uint8_t acc_settings[2] = {LSM303_CTRL_REG1_A, 0x57}; // 50Hz, Enable X,Y,Z
    HAL_I2C_Master_Transmit(&hi2c1, LSM303_ACC_ADDR, acc_settings, 2, 10);

    // 2. Initialize Gyroscope (L3GD20) via SPI
    uint8_t gyro_reg = L3GD20_CTRL_REG1;
    uint8_t gyro_settings = 0xFF; // Enable, 95Hz, Cutoff 25
    
    // SPI Write Sequence: CS Low -> Register -> Value -> CS High
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &gyro_reg, 1, 10);
    HAL_SPI_Transmit(&hspi1, &gyro_settings, 1, 10);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

void Read_Tilt_Angle(void) {
    uint8_t Rec_Data[6];
    uint8_t spi_addr;

    // --- READ ACCELEROMETER (I2C) ---
    // 0x80 | Register sets "auto-increment" to read multiple bytes
    HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDR, LSM303_OUT_X_L_A | 0x80, 1, Rec_Data, 6, 10);
    
    // Combine bytes (LSM303 is Little Endian)
    // Note: Orientation depends on mounting. Assuming standard vertical mount:
    Accel_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]); 
    Accel_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]); 
    
    // Calculate basic Accel Angle
    Accel_Angle = atan2(Accel_Y_RAW, Accel_Z_RAW) * 57.29;

    // --- READ GYROSCOPE (SPI) ---
    // Read X-axis (Pitch rate)
    spi_addr = L3GD20_OUT_X_L | 0xC0; // 0xC0 = Read bit + Auto-increment
    
    uint8_t tx_data[3] = {spi_addr, 0x00, 0x00}; // Address + 2 dummy bytes
    uint8_t rx_data[3];

    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    // Your data is now in rx_data[1] and rx_data[2]
    Gyro_X_RAW = (int16_t)(rx_data[2] << 8 | rx_data[1]);
    
    // Convert Raw to Degrees Per Second (Sensitivity ~0.00875 for 250dps scale)
    Gyro_Rate_X = (float)Gyro_X_RAW * 0.00875;

    // --- COMPLEMENTARY FILTER ---
    // Combine Gyro (fast, drifts) and Accel (slow, noisy)
    // Formula: Angle = 0.98 * (Angle + Gyro*dt) + 0.02 * Accel
    Current_Angle = 0.98 * (Current_Angle + Gyro_Rate_X * dt) + 0.02 * Accel_Angle;
}
/* USER CODE END 0 */
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  
  // --- MANUAL GPIO CONFIGURATION FOR MOTORS (PC6 & PC7) ---
  // This connects the physical pins to the Internal TIM8
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Ensure Port C clock is on


  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  Init_Sensors(); // Configures the LSM303 and L3GD20
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // Left Motor (PC6)
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Right Motor (PC7)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {

    Read_Tilt_Angle(); // Call the new function

    // --- 2. PID CALCULATION ---
    float Target_Angle = -1.5; // Mechanical balance point offset
    
    Error = Current_Angle - Target_Angle;
    Integral += Error * dt;
    Derivative = (Error - Previous_Error) / dt;

    // Anti-windup (prevents Integral from growing too large)
    if (Integral > 400) Integral = 400;
    if (Integral < -400) Integral = -400;

    PID_Total = (Kp * Error) + (Ki * Integral) + (Kd * Derivative);
    Previous_Error = Error;

    // --- 3. SAFETY & MOTOR UPDATE ---
    // Convert PID output to PWM (clamp between -1000 and 1000)
    int16_t pwm_out = constrain_val((int16_t)PID_Total, -1000, 1000);

    // Safety: If angle is too steep (>45 deg), stop motors immediately
    if (Current_Angle > 45.0 || Current_Angle < -45.0) {
        Motor_Control(0,0);
        Integral = 0; // Reset integral so it doesn't jump on recovery
    } else {
        Motor_Control(pwm_out,pwm_out);
    }


    int len = sprintf(UART_Buffer, "Ang: %.2f | Gyr: %.2f | PWM:%d\r\n", Current_Angle, Gyro_Rate_X, pwm_out);
    HAL_UART_Transmit(&huart1, (uint8_t*)UART_Buffer, len, 10);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (Current_Angle > 0) ? (uint32_t)(Current_Angle * 40) : 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (Current_Angle < 0) ? (uint32_t)(-Current_Angle * 40) : 0);
    HAL_Delay(10); // 10ms del

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
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
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */

  static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // --- PWM CONFIGURATION ---
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  // Enable Channel 2 (Left Motor - M1/M2)
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  
  // --- ENABLE CHANNEL 3 HERE (Right Motor - M3/M4) ---
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Break/DeadTime Config (Standard)
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_TIM_MspPostInit(&htim8);
}


  // static void MX_TIM8_Init(void)
// {

//   /* USER CODE BEGIN TIM8_Init 0 */
//   /* USER CODE END TIM8_Init 0 */

//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};
//   TIM_OC_InitTypeDef sConfigOC = {0};
//   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

//   /* USER CODE BEGIN TIM8_Init 1 */
//   /* USER CODE END TIM8_Init 1 */
//   htim8.Instance = TIM8;
//   htim8.Init.Prescaler = 71;
//   htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim8.Init.Period = 999;
//   htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim8.Init.RepetitionCounter = 0;
//   htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = 0;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//   if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//   sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//   sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//   sBreakDeadTimeConfig.DeadTime = 0;
//   sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//   sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//   sBreakDeadTimeConfig.BreakFilter = 0;
//   sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//   sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//   sBreakDeadTimeConfig.Break2Filter = 0;
//   sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//   if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN TIM8_Init 2 */
//   /* USER CODE END TIM8_Init 2 */
//   HAL_TIM_MspPostInit(&htim8);

// }

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */
  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */
  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */
  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L_DIR1_Pin|L_DIR2_Pin|R_DIR1_Pin|R_DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L_DIR1_Pin L_DIR2_Pin R_DIR1_Pin R_DIR2_Pin */
  GPIO_InitStruct.Pin = L_DIR1_Pin|L_DIR2_Pin|R_DIR1_Pin|R_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L_ENC_A_Pin R_ENC_A_Pin */
  GPIO_InitStruct.Pin = L_ENC_A_Pin|R_ENC_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : L_ENC_B_Pin R_ENC_B_Pin */
  GPIO_InitStruct.Pin = L_ENC_B_Pin|R_ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
