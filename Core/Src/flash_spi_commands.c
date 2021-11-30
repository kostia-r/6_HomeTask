/*
 * flash_spi_commands.c
 *
 *  Created on: Nov 29, 2021
 *      Author:  Konstantin Rudenko
 */
/* Private includes ----------------------------------------------------------*/
#include "flash_spi_commands.h"
/* Defines -----------------------------------------------------------*/
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
/* Private code ---------------------------------------------------------*/

static void flashStateSwitch(State state)
{
	if (state == ON)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // CE = LOW
	else if (state == OFF)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // CE = HIGH
}

static void flashSPIRead(SPI_HandleTypeDef *hspi, uint8_t *readBuffer, uint16_t size_readBuffer)
{
	HAL_SPI_Receive(hspi, readBuffer, size_readBuffer, 1000);
}

static void flashSPIWrite(SPI_HandleTypeDef *hspi, uint8_t *writeBuffer, uint16_t size_writeBuffer)
{
	HAL_SPI_Transmit(hspi, writeBuffer, size_writeBuffer, 1000);
}

static void flashWriteStateSwith(SPI_HandleTypeDef *hspi, State state)
{
	uint8_t command = 0;
	uint8_t command_arr[] = { 0 };
	if (state == ON)
	{
		command = ENABLE_WRITE_STATUS;
		flashStateSwitch(ON);
		flashSPIWrite(hspi, &command, sizeof(command));
		flashStateSwitch(OFF);
		command_arr[0] = WRITE_STATUS;
		command_arr[1] = 0x0;
		flashStateSwitch(ON);
		flashSPIWrite(hspi, command_arr, sizeof(command));
		flashStateSwitch(OFF);
		command = WREN;
		flashStateSwitch(ON);
		flashSPIWrite(hspi, &command, sizeof(command));
		flashStateSwitch(OFF);
	}
	else if (state == OFF)
	{
		command = WRDI;
		flashStateSwitch(ON);
		flashSPIWrite(hspi, &command, sizeof(command));
		flashStateSwitch(OFF);
	}
}

uint8_t flashGetStatus(SPI_HandleTypeDef *hspi)
{
	HAL_StatusTypeDef hal_result;
	uint8_t command[] = { READ_STATUS, 0x00 };
	uint8_t received[2];
	flashStateSwitch(ON);
	if ((hal_result = HAL_SPI_TransmitReceive(hspi, command, received, sizeof(received), 1000)) != HAL_OK)
		Error_Handler();
	flashStateSwitch(OFF);
	return received[1];
}

static void flashWaitWhileBusy(SPI_HandleTypeDef *hspi)
{
	while (flashGetStatus(hspi) & SBIT_BUSY)
		;
}

static void flashHardwareEOWStateSwitch(SPI_HandleTypeDef *hspi, State state)
{
	uint8_t command = 0;

	if (state == ON)
		command = ENABLE_SO;
	else if (state == OFF)
		command = DISABLE_SO;

	flashStateSwitch(ON);
	flashSPIWrite(hspi, &command, sizeof(command));
	flashStateSwitch(OFF);
}

void WriteByteToFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data)
{
	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { BYTE_PROG, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, *data };
	flashWriteStateSwith(hspi, ON);
	flashStateSwitch(ON);
	flashSPIWrite(hspi, writeBuffer, sizeof(writeBuffer));
	flashStateSwitch(OFF);
	flashWaitWhileBusy(hspi);
	flashWriteStateSwith(hspi, OFF);
}

void WriteToFlash(SPI_HandleTypeDef *hspi, uint32_t startAddress, uint8_t *data, uint16_t size_writeBuffer)
{
	startAddress &= 0xFFFFFF;
	uint8_t address_cmd[] = { AAI, (startAddress >> 16) & 0xFF, (startAddress >> 8) & 0xFF, startAddress & 0xFF };
	uint8_t data_cmd[3] = { AAI, 0x00, 0x00 };
	uint16_t written = 0;
	uint16_t remaining;
	flashWriteStateSwith(hspi, ON);
	flashHardwareEOWStateSwitch(hspi, ON);

	while ((remaining = size_writeBuffer - written) > 1)
	{
		flashStateSwitch(ON);
		if (!written)
			flashSPIWrite(hspi, address_cmd, sizeof(address_cmd));
		data_cmd[1] = data[written++];
		data_cmd[2] = data[written++];

		if (written <= 2)
			flashSPIWrite(hspi, data_cmd + 1, 2);
		else
			flashSPIWrite(hspi, data_cmd, 3);
		flashStateSwitch(OFF);

		flashStateSwitch(ON);
		while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
			;
		flashStateSwitch(OFF);
	}

	flashWriteStateSwith(hspi, OFF);
	flashHardwareEOWStateSwitch(hspi, OFF);

	if (remaining)
		WriteByteToFlash(hspi, startAddress + written, &data[written]);
}

void ReadByteFromFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *readBuffer)
{
	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { READ_HIGH_SPEED, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, 0xFF };

	flashStateSwitch(ON);
	flashSPIWrite(hspi, writeBuffer, sizeof(writeBuffer));
	flashSPIRead(hspi, readBuffer, 1);
	flashStateSwitch(OFF);
}

void ReadFromFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *readBuffer)
{
	address &= 0xFFFFFF;
	uint8_t writeBuffer[] = { READ_HIGH_SPEED, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF, 0xFF };
	uint8_t readTempBuffer[200];
	uint8_t count = 0;
	flashStateSwitch(ON);
	flashSPIWrite(hspi, writeBuffer, sizeof(writeBuffer));
	flashSPIRead(hspi, readTempBuffer, sizeof(readTempBuffer));
	flashStateSwitch(OFF);

	while (readTempBuffer[count] != 0xFF)
	{
		readBuffer[count] = readTempBuffer[count];
		count++;
	}
}

void ChipErase(SPI_HandleTypeDef *hspi)
{
	uint8_t command = CHIP_ERASE;
	flashWriteStateSwith(hspi, ON);
	flashStateSwitch(ON);
	HAL_SPI_Transmit(hspi, &command, sizeof(command), 1000); //send chip-erase command
	flashStateSwitch(OFF);
	flashWaitWhileBusy(hspi);
	flashWriteStateSwith(hspi, OFF);
}

void ReadID(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
	uint8_t writeBuffer[4] = { READID, 0x00, 0x00, 0x00 };
	uint8_t readBuffer[4] = { 0 };
	flashStateSwitch(ON);
	flashSPIWrite(hspi, writeBuffer, sizeof(writeBuffer));
	flashSPIRead(hspi, readBuffer, sizeof(readBuffer));
	flashStateSwitch(OFF);
	char outputStr[40] = { 0 };
	sprintf(outputStr, "Device ID: 0x%X 0x%X 0x%X 0x%X\r\n", readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);
	HAL_UART_Transmit(huart, (uint8_t*) &outputStr, sizeof(outputStr), 10);
}
