/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> //for clearing the string buffer

//don't forget to add the directory the MPU6050 files are in to your include path
#include "MPU6050.h"
#include "REG_OPTIONS.h"
#include "TEST_FUNCTIONS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRINT_BUFF_SIZE (1024)
#define HAL_I2C_TIMEOUT (100)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int stm32_reg_write_mpu6050(uint16_t regAddr, uint8_t regValue);
int stm32_reg_read_mpu6050(uint16_t regAddr, uint8_t* valAddr);
int stm32_burst_read_mpu6050(uint16_t regAddr, uint8_t* data, uint16_t bytes);
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
  char printBuff[PRINT_BUFF_SIZE]; //used by print helper functions
  uint16_t stringLen = 0; //length of string in buffer
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_REG_WRITE_TYPE* writeReg = stm32_reg_write_mpu6050;
  MPU6050_REG_READ_TYPE* readReg = stm32_reg_read_mpu6050;
  MPU6050_BURST_READ_TYPE* burstRead = stm32_burst_read_mpu6050;
  DELAY_MS_TYPE* delay = HAL_Delay;
  TIME_MS_TYPE* getTime = HAL_GetTick;

  init_mpu6050(writeReg, delay);
  

  //configure important accelerometer and gyro settings
  
  //setup the low pass filter. 
  writeReg(REG_CONFIG, DLPF_CFG_6 | EXT_SYNC_OFF);

  //select Gyroscope's full scale range
  writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);   
  
  //select Accelerometer's full scale range
  writeReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL_2G);

  //setup the sample rate divider
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);

  //read back the setup register values, and compare them to expected
  SETUP_REGISTERS expected = {
    .fsync = EXT_SYNC_OFF, 
    .dlpf = DLPF_CFG_6,
    .gyro_sel = GYRO_FS_SEL_250_DPS,
    .accel_sel = ACCEL_FS_SEL_2G,
    .rate_div = SAMPLE_RATE_100Hz
  };
  
  SETUP_REGISTERS actual = read_setup_registers(readReg);
  
  stringLen = build_setup_registers_string(expected, actual, printBuff);
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //run the factory tests for accelerometer and gyroscope, then print the results
  FACTORY_TEST_RESULTS gyroResults = gyro_self_test(readReg, writeReg, delay);
  FACTORY_TEST_RESULTS accelResults = accel_self_test(readReg, writeReg, delay);
  stringLen = build_self_tests_string(gyroResults, accelResults, printBuff);
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //run a test that polls each axis individually
  uint16_t sampleRate = 100; //Hz
  uint16_t sampleTime = 250; //ms
  int numSamples = 6 * MS_TO_S((float)sampleTime) * sampleRate;

  float data[numSamples];
  poll_axes_individually(sampleRate, sampleTime, ACCEL_FS_2_DIV, GYRO_FS_250_DIV, data, readReg, delay, getTime);
  stringLen = build_poll_axes_string(data, numSamples, printBuff);
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //fifo related tests
  uint16_t fifoCountResults[NUM_FIFO_COUNT_TESTS];
  uint32_t timePerIter[NUM_FIFO_COUNT_TESTS];
  float bytesPerRead = -1;


  //enable the fifo
  writeReg(REG_USER_CTRL, FIFO_EN);
  delay(100);
  
  //run tests on the fifo that control how much data is written
  //to the fifo in one cycle, by variying the number of axes writing
  //to the fifo and their sample rate
  
  //3 gyro axes at 100Hz for 1 sec
  writeReg(REG_FIFO_EN, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(
    1000, 
    100, 
    3, 
    burstRead, 
    readReg, 
    delay, 
    getTime, 
    fifoCountResults, 
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);
  
  //3 accel axes at 100Hz for 1 sec
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN);
  fifo_count_test(
    1000, 
    100, 
    3, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //6 axes at 100Hz for 2 sec (should fail)
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(
    2000, 
    100, 
    6, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);
  

  //3 accel axes at 100Hz for 2 sec (should fail)
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN);
  fifo_count_test(
    2000, 
    100, 
    3, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //6 axes at 100Hz for 350ms
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(
    350, 
    100, 
    6, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);
  
  //6 axes at 50Hz for 800ms
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_50Hz);
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(
    800, 
    50, 
    6, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //6 axes at 200Hz for 200ms
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_200Hz);
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(
    200, 
    200, 
    6, 
    burstRead, 
    readReg,
    delay,
    getTime,
    fifoCountResults,
    timePerIter,
    &bytesPerRead
  );
  stringLen = build_fifo_count_string(
    fifoCountResults, 
    timePerIter,
    bytesPerRead,
    printBuff
  );
  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  //seems like the fifo doesn't like to work if it gets filled with much more than 600 bytes?
  
  //test gathering data from the fifo
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN); //enable all 6 axes to be placed in fifo
  uint8_t fifoData[NUM_READ_FIFO_TESTS][FIFO_SIZE];
  uint16_t fifoCounts[NUM_READ_FIFO_TESTS];
  read_fifo_test(
    50, //really short fifo read period to keep the printed string from being too big
    fifoData,
    fifoCounts,
    burstRead, 
    readReg,
    delay,
    getTime
  ); 

  stringLen = build_read_fifo_string(
    ACCEL_FS_2_DIV, 
    GYRO_FS_250_DIV,
    fifoData,
    fifoCounts,
    printBuff
  );

  HAL_UART_Transmit(&huart2, (uint8_t*)printBuff, stringLen, 100);
  memset(printBuff, '\0', PRINT_BUFF_SIZE);

  char endString[21] = "\r\n**Tests Complete**\0";
  HAL_UART_Transmit(&huart2, (uint8_t*)endString, 21, 100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int stm32_reg_write_mpu6050(uint16_t regAddr, uint8_t regValue)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        &regValue,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    return status;
}

int stm32_reg_read_mpu6050(uint16_t regAddr, uint8_t* valAddr)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        valAddr,
        I2C_MEMADD_SIZE_8BIT,
        HAL_I2C_TIMEOUT
    );
    return status;
}

int stm32_burst_read_mpu6050(uint16_t regAddr, uint8_t* data, uint16_t bytes)
{
     HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        bytes,
        HAL_I2C_TIMEOUT
    );
    return status;
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
