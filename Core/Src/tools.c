/*
 * tools.c
 *
 *  Created on: Mar 13, 2022
 *      Author: Maryna
 */


#include "tools.h"
#include "string.h"
#include "usart.h"

void uartLog(char *message)
{

	int lenght = strlen(message);
	HAL_UART_Transmit(&huart2, (uint8_t*)message, lenght, 1000);
	//HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 1000);

}

