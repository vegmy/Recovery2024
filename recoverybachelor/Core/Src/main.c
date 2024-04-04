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
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "ms5803.h"
#include "gps.h"
#include "state.h"
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
#define I2C_ADDRESS_GPS 0x42 << 1 
#define buf_gps 100                 // Satt til å holde størrelsen til en GPGGA melding, blir justert av funksjon senere
volatile uint32_t tid;
volatile uint8_t tender_flagg = 0u;
volatile uint8_t tender_aktiv = 0u;
volatile uint8_t trans2_flagg = 0u;
volatile uint8_t trans2_reset = 0u;
volatile uint8_t trans4_flagg = 0u;
volatile uint8_t trans4_reset = 0u;
volatile uint8_t trans6_flagg = 0u;
volatile uint8_t trans6_reset = 0u;
volatile uint8_t reservekrets_flagg = 0u;
volatile uint8_t reservekrets = 0u;
volatile uint8_t eagle_flagg = 0u;
volatile uint8_t eagleaktivering = 0u;
volatile uint8_t gps_flagg;
volatile uint8_t gps_meas;
volatile uint8_t trykk_flagg;
volatile uint8_t trykk_meas;
volatile uint8_t tilstandsreg1 = 0u;
volatile uint8_t tilstandsreg2 = 0u;
volatile uint8_t separation_conf = 0u;
volatile uint8_t e_main = 0u;
volatile uint8_t e_redundant = 0u;
volatile uint8_t tender_decender = 0u;


  

volatile GPIO_PinState dio1_em1;
volatile GPIO_PinState dio2_em2;
volatile GPIO_PinState dio3_td1;
volatile GPIO_PinState dio4_td2;
volatile GPIO_PinState foto;
volatile GPIO_PinState magn;
volatile GPIO_PinState heli;



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

  
  float pressureMeasurements[NUM_MEASUREMENTS]; 
  int measurementIndex = 0; 
  float temperature, pressure;
  memset(pressureMeasurements, 0, sizeof(pressureMeasurements));
  for(int i = 1; i <= 6; i++) {
	  MS5803_coeff(&hi2c1, &MS5803_coefficient[i-1], i); //get coefficients
  } 
  uint8_t disableGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A}; // slår av GPGLL NEMA setninger
  uint8_t disableGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31}; // slår av GPGSA NEMA setninger
  uint8_t disableGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38}; // slår av GPGSV NEMA setninger
  uint8_t disableRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F}; // slår av GPRMC NEMA setninger
  uint8_t disableVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46}; // slår av GPVTG NEMA setninger
  //uint8_t increaseRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; // øker frekvensen av målinger til 10Hz fra 1Hz
  uint8_t increaseRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; // øker frekvensen av målinger til 5Hz fra 1Hz

  uint8_t gpsMessage[buf_gps]; // Buffer for å lagre mottatte gps meldinger
  uint16_t messageSize = buf_gps; // Setter variabel til størrelsen til gps buffer
  char resultBuffer[30]; // Buffer for å oppholde utvalgt informasjon fra GPGGA melding
  char statebuffer[2];
  uint8_t presbuf[4];
  uint8_t communicationBuf[38];
  
  // Sender UBX meldinger definert over for å slå av alle setninger untatt GPGGA, samt øker raten til 5Hz
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, disableGLL, sizeof(disableGLL), 1000); 
  HAL_Delay(5);
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, disableGSA, sizeof(disableGSA), 1000); 
  HAL_Delay(5);
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, disableGSV, sizeof(disableGSV), 1000); 
  HAL_Delay(5);
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, disableRMC, sizeof(disableRMC), 1000); 
  HAL_Delay(5);
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, disableVTG, sizeof(disableVTG), 1000); 
  HAL_Delay(5);
  HAL_I2C_Master_Transmit(&hi2c1, 0x84, increaseRate, sizeof(increaseRate), 1000); 
  HAL_Delay(5);

  // Test av kommuniksjon mot trykksensor og gps, inkludert resett av trykksensor før oppstart.
  uint8_t gps_address = (0x42 << 1);
  HAL_I2C_IsDeviceReady(&hi2c1, gps_address, 10, 1000);
  HAL_StatusTypeDef test = MS5803_reset(&hi2c1); //reset
  uint8_t pres_address = (0x76 << 1);
  HAL_I2C_IsDeviceReady(&hi2c1, pres_address, 10, 1000);
  // kansje legg inn en UBX melding for resetting av gps helt i starten av programmet???
  gps_flagg = 1;
  gps_meas = 0u;
  trykk_flagg = 1;
  trykk_meas = 0u;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while (1)
  // {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // }
  while (1)
  {
  tid = HAL_GetTick(); //kan fjernes
  
  // måling av tilstand for alle digitale signal i systemet
    dio1_em1 = HAL_GPIO_ReadPin(GPIO_IN_DIO1_GPIO_Port,GPIO_IN_DIO1_Pin); // Måling av verdi fra diode 1 som verifiserer kondensator 1 over 7.3V
    dio2_em2 = HAL_GPIO_ReadPin(GPIO_IN_DIO2_GPIO_Port,GPIO_IN_DIO2_Pin); // Måling av verdi fra diode 2 som verifiserer kondensator 2 over 7.3V
    dio3_td1 = HAL_GPIO_ReadPin(GPIO_IN_DIO3_GPIO_Port,GPIO_IN_DIO3_Pin); // Måling av verdi fra diode 3 som verifiserer kondensator 3 over 7.3V
    dio4_td2 = HAL_GPIO_ReadPin(GPIO_IN_DIO4_GPIO_Port,GPIO_IN_DIO4_Pin); // Måling av verdi fra diode 4 som verifiserer kondensator 4 over 7.3V
    foto = HAL_GPIO_ReadPin(GPIO_IN_FOTO_GPIO_Port,GPIO_IN_FOTO_Pin); // Måling av verdi fra fototransistor
    magn = HAL_GPIO_ReadPin(GPIO_IN_MAGN_GPIO_Port,GPIO_IN_MAGN_Pin); // Måling av verdi fra magnetbryter
    heli = HAL_GPIO_ReadPin(GPIO_IN_HELI_GPIO_Port,GPIO_IN_HELI_Pin); // Måling av verdi helikopterplugg

  
   // Aktivering av eagles dersom helikopterplugg blir trukket
    if ( (heli == GPIO_PIN_RESET) && (eagleaktivering == 0) ) // Eagle 1
    {
      HAL_GPIO_WritePin(GPIO_OUT_TRANS2_GPIO_Port,GPIO_OUT_TRANS2_Pin,GPIO_PIN_SET); // Setter utgangsignal til gate for transistor 2 som styrer eagle krets 1
      
      tender_flagg = 1; // Aktiverer 15 sekunders timer til aktivering av tender decender krets
      trans2_flagg = 1; // Aktiverer 2 sekunders timer som vil slå av transistor igjen
      reservekrets_flagg = 1;  // Aktiverer 3 sekunders timer til aktivering av eagle krets 2
      e_main = 1;
      eagleaktivering++; // sikkrer at transistor 2 og 4 ikke blir aktivert mer enn en gang, grunnet at helikopterplugg er resett helt fra drop
    }
    if (reservekrets) // Setter utgangsignal til gate for transistor 4 som styrer eagle krets 2
    {
      HAL_GPIO_WritePin(GPIO_OUT_TRANS4_GPIO_Port,GPIO_OUT_TRANS4_Pin,GPIO_PIN_SET); 
      trans4_flagg = 1;
      e_redundant = 1;
      reservekrets_flagg = 0; 
      reservekrets = 0;
    }
    if (tender_aktiv) // Setter utgangsignal til gate for transistor 6 som styrer tender decenderkrets med kondensator 3 og 4
    { 
      HAL_GPIO_WritePin(GPIO_OUT_TRANS6_GPIO_Port,GPIO_OUT_TRANS6_Pin,GPIO_PIN_SET);
      trans6_flagg = 1;
      tender_decender = 1;
      tender_flagg = 0;
      tender_aktiv = 0;
    }
    if((foto == GPIO_PIN_SET) && (magn == GPIO_PIN_RESET))
    {
      separation_conf = 1;
    }
    if (trans2_reset) // Slår av transistor 2
    {  
      HAL_GPIO_WritePin(GPIO_OUT_TRANS2_GPIO_Port,GPIO_OUT_TRANS2_Pin,GPIO_PIN_RESET);
      trans2_flagg = 0;
      trans2_reset = 0;
    }
    if (trans4_reset) // Slår av transistor 4
    {  
      HAL_GPIO_WritePin(GPIO_OUT_TRANS4_GPIO_Port,GPIO_OUT_TRANS4_Pin,GPIO_PIN_RESET);
      trans4_flagg = 0;
      trans4_reset = 0;
    }
    if (trans6_reset) // Slår av transistor 6
    { 
      HAL_GPIO_WritePin(GPIO_OUT_TRANS8_GPIO_Port,GPIO_OUT_TRANS8_Pin,GPIO_PIN_RESET);
      trans6_flagg = 0;
      trans6_reset = 0;
    }
    if(trykk_meas) // tar trykkmålig hver 20ms
    {
      trykk_meas = 0;
      trykk_flagg = 0;
      MS5803_get_values(&hi2c1, ADC_256, &temperature, &pressure); 
        if (measurementIndex < NUM_MEASUREMENTS) 
        {
          pressureMeasurements[measurementIndex] = pressure; // Store the pressure measurement
          measurementIndex++; // Move to the next index for the next measurement
          pressTobytes(pressure, presbuf);
        } 
        else 
        {
         measurementIndex = 0; // Uncomment this line to start measurements over
        }
        if (pressure > 2000.f)
        {
          eagle_flagg = 1;
        }
      trykk_flagg = 1;
    }
  
 
    if (gps_meas) // tar gps måling hver 100ms
    {
      gps_meas = 0;
      gps_flagg = 0;
      messageSize = buf_gps;
      memset(gpsMessage, 0, messageSize); // Resetter buffer før mottak av ny melding
      HAL_StatusTypeDef status = readGPSMessage(&hi2c1, gpsMessage, &messageSize);
      
    if ((status == HAL_OK) && (messageSize > 60)) 
    {
     // Successfully read a GPS message, process it
    processGPSMessage((char*)gpsMessage, resultBuffer, sizeof(resultBuffer)); 
        
    } 
      
    gps_flagg = 1;
    }
    // oppsett av tilstandsregister 1 og 2, tilbakemelding til FC
    stateControll(&tilstandsreg1, &tilstandsreg2, statebuffer, sizeof(statebuffer));
    memset(communicationBuf, 0, 38); // Resetter buffer før mottak av ny melding
    snprintf(communicationBuf, 38, "%s,%s,%s", resultBuffer, presbuf, statebuffer);
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
