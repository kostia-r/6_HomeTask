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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIME_CAPSULE_WRITE_MODE 0//<<======!!!!! UNCOMMENT THIS DEFINE TO ENABLE TIME CAPSULE WRITING MODE!!!!!!
#define NUM_STRINGS 13      //size of Time Capsule array
#define LENGTH_OFSTRING 100 //---//----

#define READID 				0x90 //read-ID command
#define WREN 				0x06 //write-enable command
#define WRDI				0x04 //write-disable command
#define ENABLE_WRITE_STATUS	0x50
#define WRITE_STATUS		0x01
#define BYTE_PROG 			0x02 //one byte-program command
#define READ 				0x03 //read command
#define READ_HIGH_SPEED		0x0B //High-Speed Read command
#define SBIT_BUSY			0x01
#define READ_STATUS			0x05
#define AAI					0xAD //Auto Address Increment Programming command
#define ENABLE_SO			0x70
#define DISABLE_SO			0x80
#define CHIP_ERASE 			0x60 //chip erase command: clear all bits of devise to 0xFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void flashStart(void) // CE = LOW
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

void flashStop(void) // CE = HIGH
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

void flashSPIRead(uint8_t *readBuffer, uint16_t size_readBuffer)
{
	HAL_SPI_Receive(&hspi1, readBuffer, size_readBuffer, 1000);
}

void flashSPIWrite(uint8_t *writeBuffer, uint16_t size_writeBuffer)
{
	HAL_SPI_Transmit(&hspi1, writeBuffer, size_writeBuffer, 1000);
}

void flashSPIReadWrite(uint8_t *readBuffer, uint8_t *writeBuffer, uint16_t size_readBuffer, uint16_t size_writeBuffer)
{
	HAL_SPI_TransmitReceive(&hspi1, writeBuffer, readBuffer, (size_readBuffer + size_writeBuffer), 1000);
}

void flashWriteEnable(void)
{
	flashStart();
	uint8_t command = WREN;
	flashSPIWrite(&command, sizeof(command));
	flashStop();
}

void flashWriteDisable(void)
{
	flashStart();
	uint8_t command = WRDI;
	flashSPIWrite(&command, sizeof(command));
	flashStop();
}

void flashEnableWriteStatus(void)
{
	uint8_t command = ENABLE_WRITE_STATUS;
	flashSPIWrite(&command, sizeof(command));
}

void flashWriteStatus(void)
{
	flashStart();
	flashEnableWriteStatus();
	flashStop();
	uint8_t command[] = { WRITE_STATUS, 0x0 };
	flashStart();
	flashSPIWrite(command, sizeof(command));
	flashStop();
}

uint8_t flashGetStatus(void)
{
	HAL_StatusTypeDef hal_result;
	flashStart();
	uint8_t command[] = { READ_STATUS, 0x00 };
	uint8_t received[2];
	if ((hal_result = HAL_SPI_TransmitReceive(&hspi1, command, received, sizeof(received), 1000)) != HAL_OK)
		Error_Handler();
	flashStop();
	return received[1];
}

void flashWaitWhileBusy(void)
{
	while (flashGetStatus() & SBIT_BUSY)
		;
}

void flashEnableHardwareEOW(void)
{
	flashStart();
	uint8_t command = ENABLE_SO;
	flashSPIWrite(&command, sizeof(command));
	flashStop();
}

void flashDisableHardwareEOW(void)
{
	flashStart();
	uint8_t command = DISABLE_SO;
	flashSPIWrite(&command, sizeof(command));
	flashStop();
}

void WriteByteToFlash(uint32_t address, uint8_t *data)
{
	flashWriteStatus();
	flashWriteEnable();

	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { BYTE_PROG, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, *data };
	flashStart();
	flashSPIWrite(writeBuffer, sizeof(writeBuffer));
	flashStop();

	flashWaitWhileBusy();
	flashWriteDisable();
}

void WriteToFlash(uint32_t startAddress, uint8_t *data, uint16_t size_writeBuffer)
{
	flashWriteStatus();
	flashWriteEnable();
	flashEnableHardwareEOW();

	startAddress &= 0xFFFFFF;
	uint8_t address_cmd[] = { AAI, (startAddress >> 16) & 0xFF, (startAddress >> 8) & 0xFF, startAddress & 0xFF };
	uint8_t data_cmd[3] = { AAI, 0x00, 0x00 };
	uint16_t written = 0;
	uint16_t remaining;

	while ((remaining = size_writeBuffer - written) > 1)
	{
		flashStart();
		if (!written)
			flashSPIWrite(address_cmd, sizeof(address_cmd));
		data_cmd[1] = data[written++];
		data_cmd[2] = data[written++];

		if (written <= 2)
			flashSPIWrite(data_cmd + 1, 2);
		else
			flashSPIWrite(data_cmd, 3);
		flashStop();

		flashStart();
		while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
			;
		flashStop();
	}

	flashWriteDisable();

	flashDisableHardwareEOW();
	if (remaining)
		WriteByteToFlash(startAddress + written, &data[written]);
}

void ReadByteFromFlash(uint32_t address, uint8_t *readBuffer, uint16_t size_readBuffer)
{
	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { READ_HIGH_SPEED, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, 0xFF };

	flashStart();
	flashSPIWrite(writeBuffer, sizeof(writeBuffer));
	flashSPIRead(readBuffer, size_readBuffer);
	flashStop();
}

void ReadFromFlash(uint32_t address, uint8_t *readBuffer)
{
	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { READ_HIGH_SPEED, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, 0xFF };
	uint8_t readTempBuffer[200];
	uint8_t count = 0;
	flashStart();
	flashSPIWrite(writeBuffer, sizeof(writeBuffer));
	flashSPIRead(readTempBuffer, sizeof(readTempBuffer));
	flashStop();
	while (readTempBuffer[count] != 0xFF)
	{
		readBuffer[count] = readTempBuffer[count];
		count++;
	}
}

void ChipErase(void)
{
	flashWriteStatus();
	flashWriteEnable();

	uint8_t command = CHIP_ERASE;
	flashStart();
	HAL_SPI_Transmit(&hspi1, &command, sizeof(command), 1000); //send chip-erase command
	flashStop();

	flashWaitWhileBusy();
	flashWriteDisable();
}

void ReadID(void)
{
	uint8_t writeBuffer[4] = { READID, 0x00, 0x00, 0x00 };
	uint8_t readBuffer[4] = { 0 };
	flashStart();
	flashSPIWrite(writeBuffer, sizeof(writeBuffer));
	flashSPIRead(readBuffer, sizeof(readBuffer));
	flashStop();
	char outputStr[40] = { 0 };
	sprintf(outputStr, "Device ID: 0x%X 0x%X 0x%X 0x%X\r\n", readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);
	HAL_UART_Transmit(&huart3, (uint8_t*) &outputStr, sizeof(outputStr), 10);
}

void UARTClearScreen(void)
{
	HAL_UART_Transmit(&huart3, (uint8_t*) "\x1B\x5B\x32\x4A\r\n", 6, 10);
}
/* USER CODE END 0 */

/**
 * @brief  The applic"ation entry point.
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
	MX_SPI1_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	uint32_t address = 0x000000;

#ifdef TIME_CAPSULE_WRITE_MODE //create Time Capsule array and write to flash memory, start at 0x000000

	char timeCapsule[NUM_STRINGS][LENGTH_OFSTRING] = { };
	strcat(timeCapsule[0], "From: Konstantin Rudenko, rudenko.x64@gmail.com\r\n");
	strcat(timeCapsule[1], "Mentor: Vitalii Kostiuk, vitalii.kostiuk@globallogic.com\r\nDate: ");
	char currentDate[30] = __DATE__;
	strcat(currentDate, "\r\n");
	strcat(timeCapsule[2], currentDate);
	strcat(timeCapsule[3], "TIME CAPSULE:\r\n");
	strcat(timeCapsule[4], "We don't need no education\r\n");
	strcat(timeCapsule[5], "We don't need no thought control\r\n");
	strcat(timeCapsule[6], "No dark sarcasm in the classroom\r\n");
	strcat(timeCapsule[7], "Teacher, leave those kids alone\r\n");
	strcat(timeCapsule[8], "Hey, Teacher, leave those kids alone!\r\n");
	strcat(timeCapsule[9], "All in all it's just another brick in the wall\r\n");
	strcat(timeCapsule[10], "All in all you're just another brick in the wall\r\n");
	strcat(timeCapsule[11], "\r\n");
	strcat(timeCapsule[12], "                     Pink Floyd - Another Brick In the Wall\r\n");
	ChipErase(); //erase all flash memory
	for (uint8_t i = 0; i < NUM_STRINGS; i++)
	{
		char writeStr[] = { };
		strcpy(writeStr, &timeCapsule[i][0]);
		WriteToFlash(address, (uint8_t*) writeStr, strlen(writeStr));
		address += 0x1000;  //increment address to next 4 KB block
	}
	HAL_Delay(1000); //this delay is needed for a more understandable visual display on the analyzer diagram
#endif

	//Read Time capsule from flash memory, start at 0x000000
	address = 0x000000;
	UARTClearScreen();
	for (uint8_t i = 0; i < NUM_STRINGS; i++)
	{
		char readStr[LENGTH_OFSTRING] = { };
		ReadFromFlash(address, (uint8_t*) readStr);
		HAL_UART_Transmit(&huart3, (uint8_t*) readStr, strlen(readStr), 10);
		address += 0x1000;  //increment address to next 4 KB block
	}

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin : PD7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
