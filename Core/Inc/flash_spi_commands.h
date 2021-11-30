/*
 * flash_spi_commands.h
 *
 *  Created on: Nov 29, 2021
 *      Author: krudenko
 */

#ifndef INC_FLASH_SPI_COMMANDS_H_
#define INC_FLASH_SPI_COMMANDS_H_

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/

void WriteByteToFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data);
void WriteToFlash(SPI_HandleTypeDef *hspi, uint32_t startAddress, uint8_t *data, uint16_t size_writeBuffer);
void ReadByteFromFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *readBuffer);
void ReadFromFlash(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *readBuffer);
void ChipErase(SPI_HandleTypeDef *hspi);
void ReadID(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

extern void Error_Handler(void);

/* typedef ---------------------------------------------*/

typedef enum
{
	ON, OFF
} State;

#endif /* INC_FLASH_SPI_COMMANDS_H_ */
