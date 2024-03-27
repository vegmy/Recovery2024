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
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ms5803.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
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

/* USER CODE BEGIN PV */
uint16_t MS5803_coefficient[6];
#define NUM_MEASUREMENTS 1200
  volatile uint8_t tellerflagg = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  float pressureMeasurements[NUM_MEASUREMENTS]; // Array to store pressure measurements
  int measurementIndex = 0; // Index for the next measurement to be stored
  uint32_t lastMeasurementTime = 0; // Timestamp of the last measurement
  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
  //while(1){
    //volatile GPIO_PinState proxvalue = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7); // Måling av verdi fra magnetsensor

  //} 
    //HAL_StatusTypeDef test = MS5803_reset(&hi2c1); //reset
    //uint8_t gps_address = (0x42 << 1);
    //HAL_StatusTypeDef test_device = HAL_I2C_IsDeviceReady(&hi2c1, gps_address, 10, 1000);
    uint8_t dev_address = (0x76 << 1);
    HAL_StatusTypeDef test_device = HAL_I2C_IsDeviceReady(&hi2c1, dev_address, 10, 1000);
  while(test_device == HAL_ERROR) {
	  // if the test fails
  }
  for(int i = 1; i <= 6; i++) {
	      MS5803_coeff(&hi2c1, &MS5803_coefficient[i-1], i); //get coefficients
        } 
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char print[32];
    //float temperature = 0.0f;
    //float pressure = 0.0f;
    float temperature, pressure;
    memset(pressureMeasurements, 0, sizeof(pressureMeasurements));
  // while (1)
  // {
  //   MS5803_get_values(&hi2c1, ADC_256, &temperature, &pressure);
  //   HAL_Delay(10);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // }
  while (1)
  {

  volatile GPIO_PinState dio1_em1 = HAL_GPIO_ReadPin(GPIO_IN_DIO1_GPIO_Port,GPIO_IN_DIO1_Pin); // Måling av verdi fra d1
  volatile GPIO_PinState dio2_em2 = HAL_GPIO_ReadPin(GPIO_IN_DIO2_GPIO_Port,GPIO_IN_DIO2_Pin); // Måling av verdi fra d2
  volatile GPIO_PinState dio3_td1 = HAL_GPIO_ReadPin(GPIO_IN_DIO3_GPIO_Port,GPIO_IN_DIO3_Pin); // Måling av verdi fra d3
  volatile GPIO_PinState dio4_td2 = HAL_GPIO_ReadPin(GPIO_IN_DIO4_GPIO_Port,GPIO_IN_DIO4_Pin); // Måling av verdi fra d4
  volatile GPIO_PinState foto = HAL_GPIO_ReadPin(GPIO_IN_FOTO_GPIO_Port,GPIO_IN_FOTO_Pin); // Måling av verdi fra fototransistor
  volatile GPIO_PinState magn = HAL_GPIO_ReadPin(GPIO_IN_MAGN_GPIO_Port,GPIO_IN_MAGN_Pin); // Måling av verdi fra magnetbryter

  volatile GPIO_PinState heli = HAL_GPIO_ReadPin(GPIO_IN_HELI_GPIO_Port,GPIO_IN_HELI_Pin); // avlesning av Miso(PB4) pin og setting av trans 4
    if (magn == GPIO_PIN_SET && foto == GPIO_PIN_RESET && heli == GPIO_PIN_RESET){
      HAL_GPIO_WritePin(GPIO_OUT_TRANS2_GPIO_Port,GPIO_OUT_TRANS2_Pin,GPIO_PIN_SET);
    }else{  
      HAL_GPIO_WritePin(GPIO_OUT_TRANS2_GPIO_Port,GPIO_OUT_TRANS2_Pin,GPIO_PIN_RESET);
    }
    if (magn == GPIO_PIN_RESET && foto == GPIO_PIN_SET && heli == GPIO_PIN_SET){ 
      //tellerflagg = 1;
      HAL_GPIO_WritePin(GPIO_OUT_TRANS8_GPIO_Port,GPIO_OUT_TRANS8_Pin,GPIO_PIN_SET);
    }else { 
      HAL_GPIO_WritePin(GPIO_OUT_TRANS8_GPIO_Port,GPIO_OUT_TRANS8_Pin,GPIO_PIN_RESET);
    }
  MS5803_get_values(&hi2c1, ADC_4096, &temperature, &pressure);
      if (measurementIndex < NUM_MEASUREMENTS) {
        pressureMeasurements[measurementIndex] = pressure; // Store the pressure measurement
        measurementIndex++; // Move to the next index for the next measurement
      } else {
        measurementIndex = 0; // Uncomment this line to start measurements over
        
      }
      lastMeasurementTime = HAL_GetTick(); // Update the timestamp of the last measurement
    }
    
    // Other tasks can be done here
  }
  /* USER CODE END 3 */
//}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */

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
