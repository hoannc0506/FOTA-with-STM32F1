/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ADDRESS_START_Current_APPLICATION 0x08005000u
#define ADDRESS_START_FOTA_APPLICATION 0x08011800u
#define Boot_Status_Address 0x0801FC00u

void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
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
  /* USER CODE BEGIN 2 */

  uint32_t ADDRESS_START_APPLICATION;
  uint32_t BootStatus = 0;

  Flash_Read_Data(Boot_Status_Address, &BootStatus, 1);

  if(BootStatus == 0x01u)
  {
	  ADDRESS_START_APPLICATION = ADDRESS_START_FOTA_APPLICATION;
  }
  else
  {
	  ADDRESS_START_APPLICATION = ADDRESS_START_Current_APPLICATION;
  }

  /* Turn off Peripheral, Clear Interrupt Flag*/
  HAL_RCC_DeInit();
  /* Clear Pending Interrupt Request, turn  off System Tick*/
  HAL_DeInit();
  /* Turn off fault harder*/
  /* Refer to PM0056 STM32F1 page 141/156:
   * The SHCSR enables the system handlers, and indicates:
   * Bit 18: USGFAULTENA-> Usage fault enable bit, set to 1 to enable
   * Bit 17: BUSFAULTENA-> Bus fault enable bit, set to 1 to enable
   * Bit 16: MEMFAULTENA-> Memory management fault enable bit, set to 1 to enable
   * To turn off these fault harder and don't change another bit, we just need to do:
   * 	SCB->SHCSR &= ~(0x111UL<<16)
   * The below line of code does the same work
   * */
//  SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk|SCB_SHCSR_BUSFAULTENA_Msk|SCB_SHCSR_MEMFAULTENA_Msk);
  SCB->SHCSR &= ~(0x0111UL<<16);
  /* The Reset_Handler was modified in file starup_stm32f103xx.s as weak function
   * We can override it to use our Reset_Handler() function
   *
   * To jump to another firmware called NewFirmware in flash memory, we need to set
   * the processor's PC at the (ADDRESS_START_NewFirmware + 4). Why ?
   * In case that the memory start at 0x0000_0000
   * when ARM Cortex processor is turned on or reset, the processor fetches two words located at 0x0000_0000
   * and 0x0000_0004 in memory. The processor uses the word located at 0x00000000 to initialize
   * the main stack pointer (MSP), and the other one at 0x00000004 to set up the program counter (PC)
   * The word stored at 0x00000004 is the memory address of the Reset_Handler() procedure,
   * which is determined by the compiler and link script. Typically, Reset_Handler calls main(),
   * which is the user's application code. After PC is initialized, the program begins execution.
   * */
  /* Set Main Stack Pointer*/
  __set_MSP(*((volatile uint32_t*) ADDRESS_START_APPLICATION));

  uint32_t JumpAddress = *((volatile uint32_t*) (ADDRESS_START_APPLICATION + 4));

  /* Set Program Counter to Blink LED Application Address*/

  void (*Reset_Handler)(void) = (void*)JumpAddress;
  Reset_Handler();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
