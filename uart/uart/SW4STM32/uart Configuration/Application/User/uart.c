/*
 * uart.c
 *
 *  Created on: 17 џэт. 2016 у.
 *      Author: vetal
 */

#include "uart.h"
#include "stdlib.h"

static uint8_t rxBuffer;
static uint8_t *rxString;
static uint8_t rxIndex;
static osThreadId *commandQueue;
static UART_HandleTypeDef *huart;

void startUART_DMA_Receiver(UART_HandleTypeDef *hUART, osMessageQId *cmdQueue) {
	commandQueue = cmdQueue;
	huart = hUART;

	__HAL_UART_FLUSH_DRREGISTER(hUART);

	HAL_UART_Receive_DMA(hUART, &rxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_FLUSH_DRREGISTER(huart);

	if (rxBuffer == '\n' || rxBuffer == '\r') {
		//enter

		if (rxIndex == 0) {
			//there is no command. rxString is empty. Ignore it
		} else {
			//set string end
			rxString[rxIndex++] = '\n';
			rxString[rxIndex++] = 0;

			//send to queue
			xQueueSendFromISR(*commandQueue, &rxString, 0);

			//remove rxString buffer

			rxIndex = 0;
			rxString = NULL;
		}
	} else {
		if (rxString == NULL) {
			//allocate new string buffer

			rxString = malloc(MAX_STR_LENGTH + 1);
		}

		rxString[rxIndex] = rxBuffer;

		rxIndex++;

		if (rxIndex > MAX_STR_LENGTH) {
			rxIndex = 0;
		}
	}
}
