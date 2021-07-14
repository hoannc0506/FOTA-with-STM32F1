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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAXSIZE 12288
#define ADDRESS_START_CURRENT_FIRMWARE 0x08005000u
#define	BOOT_STATUS_ADDRESS 0x0801FC00u

uint8_t dataBuffer[MAXSIZE];
static uint32_t GetPage(uint32_t Address);
uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		uint32_t wordCnt = (Size / 4) + ((Size % 4) != 0);
		uint32_t bootState = 0x0u;
		if(HAL_CRC_Calculate(&hcrc, (uint32_t *)dataBuffer, wordCnt) == 0x0u)
		{
			uint8_t CRC_respond[] = "Receive firmware successfully!\r\nUpdating firmware\r\n";
			HAL_UART_Transmit(&huart1, CRC_respond, sizeof(CRC_respond), HAL_MAX_DELAY);

			Flash_Write_Data(ADDRESS_START_CURRENT_FIRMWARE, (uint32_t *)dataBuffer, wordCnt);
			Flash_Write_Data(BOOT_STATUS_ADDRESS, &bootState, 1);

			HAL_NVIC_SystemReset();
		}
		else
		{
			uint8_t CRC_respond[] = "Receive firmware fail!\r\nCan't update firmware\r\n";

			HAL_UART_Transmit(&huart1, CRC_respond, sizeof(CRC_respond), HAL_MAX_DELAY);
			Flash_Write_Data(BOOT_STATUS_ADDRESS, &bootState, 1);

			HAL_NVIC_SystemReset();
		}

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
	SCB->VTOR = (uint32_t) 0x08011800;
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
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  uint8_t notification_str[] = "Send new firmware (file.bin) you want to update.\r\n\
								(The maximum size of firmware must be smaller than 12KB)\r\n";

  HAL_UART_Transmit(&huart1, notification_str, sizeof(notification_str), HAL_MAX_DELAY);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dataBuffer, MAXSIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

// Get the page that contain the specific address
static uint32_t GetPage(uint32_t Address)
{
  for (int indx=0; indx<128; indx++)
  {
	  if((Address < (0x08000000 + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (0x08000000 + FLASH_PAGE_SIZE*indx)))
	  {
		  return (0x08000000 + FLASH_PAGE_SIZE*indx);
	  }
  }

  return 0;
}

// Program Flash memory with start page address
uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*4;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
