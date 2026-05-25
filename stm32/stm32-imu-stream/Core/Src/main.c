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
#include "stdio.h"
#include "serial.h"
#include "sensor_config.h"   /* sensor / bus / mag selection — edit that file */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* See Core/Inc/sensor_config.h */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* IMU handle */
#if defined(SENSOR_MPU6050)
    static mpu6050_handle_t    imu;
#elif defined(SENSOR_MPU9250)
    static mpu9250_handle_t    imu;
#elif defined(SENSOR_ICM42688)
    static icm42688_handle_t   imu;
#elif defined(SENSOR_LSM6DSO)
    static lsm6dso_handle_t    imu;
#elif defined(SENSOR_ISM330DHCX)
    static ism330dhcx_handle_t imu;
#elif defined(SENSOR_ICM20948)
    static icm20948_handle_t   imu;
#endif

/* Standalone magnetometer handle */
#if defined(MAG_LIS3MDL)
    static lis3mdl_handle_t    mag_handle;
#elif defined(MAG_MMC5983MA)
    static mmc5983ma_handle_t  mag_handle;
#endif

#if MAG_DEFINED
static uint8_t mag_ok = 0;
#endif

static imu_raw_data_t imu_data;
static mag_raw_data_t mag_data;
uint8_t uart_data[20];

/* I2C bus contexts — 7-bit address shifted left by 1 for STM32 HAL */
#if defined(SENSOR_BUS_I2C)
    static uint16_t imu_i2c_addr = SENSOR_I2C_ADDR;
#endif
#if defined(SENSOR_MPU9250) && defined(SENSOR_BUS_I2C)
    static uint16_t ak_i2c_addr  = 0x0C;   /* AK8963 on bypass bus */
#endif
#if MAG_STANDALONE
    static uint16_t mag_i2c_addr = MAG_I2C_ADDR;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        IMU_ReadRaw((imu_handle_t *)&imu, &imu_data);
#if MAG_DEFINED
        if (mag_ok)
            MAG_ReadCorrected(MAG_HANDLE, &mag_data);
#endif
        serial_send_packet(&imu_data, &mag_data);
    }
}

static int8_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len, void *ctx)
{
    uint16_t addr = *(uint16_t *)ctx;
    addr = (addr << 1) + 1;
    if (HAL_I2C_Master_Transmit(&hi2c1, addr, &reg, 1, 100) != HAL_OK) return -1;
    if (HAL_I2C_Master_Receive (&hi2c1, addr, buf, len, 100) != HAL_OK) return -1;
    return 0;
}

static int8_t i2c_write(uint8_t reg, const uint8_t *buf, uint16_t len, void *ctx)
{
    uint16_t addr = *(uint16_t *)ctx;
    uint8_t  tx[15];   /* 1 reg byte + up to 32 data bytes */
    addr <<= 1;
    if (len > 15) return -1;
    tx[0] = reg;
    for (uint16_t i = 0; i < len; i++) tx[i + 1] = buf[i];
    return HAL_I2C_Master_Transmit(&hi2c1, addr, tx, len + 1, 100) == HAL_OK ? 0 : -1;
}

/* SPI read: assert CS, send register with read-bit set, receive data, deassert CS */
static int8_t spi2_read(uint8_t reg, uint8_t *buf, uint16_t len, void *ctx)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)ctx;
    uint8_t tx = 0x80 | reg;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin,
			GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &tx, 1, 10);
    HAL_SPI_Receive(hspi, buf, len, 10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin,
			GPIO_PIN_SET);
    return 0;
}

/* SPI write: assert CS, send register address, send data bytes, deassert CS */
static int8_t spi2_write(uint8_t reg, const uint8_t *buf, uint16_t len, void *ctx)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)ctx;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin,
			GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &reg, 1, 10);
    HAL_SPI_Transmit(hspi, (uint8_t *)buf, len, 10);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin,
			GPIO_PIN_SET);
    return 0;
}
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
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* Resolve bus callbacks and context */
#if defined(SENSOR_BUS_SPI)
  #define IMU_READ_CB   spi2_read
  #define IMU_WRITE_CB  spi2_write
  #define IMU_BUS_CTX   ((void *)&hspi2)
  #define IMU_SPI_MODE  1
#else  /* I2C or I2C_ALT */
  #define IMU_READ_CB   i2c_read
  #define IMU_WRITE_CB  i2c_write
  #define IMU_BUS_CTX   ((void *)&imu_i2c_addr)
  #define IMU_SPI_MODE  0
#endif

  /* IMU HandleInit */
#if defined(SENSOR_MPU6050)
  MPU6050_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX,
                     MPU6050_ADDR_LOW, CFG_ACCEL_RANGE, CFG_GYRO_RANGE);
#elif defined(SENSOR_MPU9250)
  MPU9250_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX, &ak_i2c_addr,
                     CFG_ACCEL_RANGE, CFG_GYRO_RANGE);
#elif defined(SENSOR_ICM42688)
  ICM42688_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX,
                      CFG_ACCEL_RANGE, CFG_GYRO_RANGE);
#elif defined(SENSOR_LSM6DSO)
  LSM6DSO_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX,
                     CFG_ACCEL_RANGE, CFG_GYRO_RANGE);
#elif defined(SENSOR_ISM330DHCX)
  ISM330DHCX_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX,
                        CFG_ACCEL_RANGE, CFG_GYRO_RANGE);
#elif defined(SENSOR_ICM20948)
  ICM20948_HandleInit(&imu, IMU_READ_CB, IMU_WRITE_CB, HAL_Delay, IMU_BUS_CTX,
                      CFG_ACCEL_RANGE, CFG_GYRO_RANGE, IMU_SPI_MODE);
#endif

  if (IMU_Init((imu_handle_t *)&imu) != IMU_OK) Error_Handler();
  IMU_CalibrateGyro((imu_handle_t *)&imu, 100);

  /* Standalone magnetometer init */
#if MAG_STANDALONE
  #if defined(MAG_LIS3MDL)
    LIS3MDL_HandleInit(&mag_handle, i2c_read, i2c_write, HAL_Delay,
                       &mag_i2c_addr, LIS3MDL_FS_4G);
  #elif defined(MAG_MMC5983MA)
    MMC5983MA_HandleInit(&mag_handle, i2c_read, i2c_write, HAL_Delay,
                         &mag_i2c_addr, MMC5983MA_BW_200HZ);
  #endif
  if (MAG_Init(MAG_HANDLE) == IMU_OK) {
      mag_ok = 1;
      MAG_SetBias(MAG_HANDLE, 1450, -1900, 2500); /* replace with measured hard-iron biases */
      /* LD2 solid ON  = mag init succeeded */
  }
#elif SENSOR_HAS_MAG
  mag_ok = 1;
  MAG_SetBias(MAG_HANDLE,136 , -63, -125);     /* replace with measured hard-iron biases */
#endif
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

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
  hi2c1.Init.Timing = 0x10D19EE6;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
int _write(int le, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
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
