/*
 * command.c
 *
 *  Created on: 17 џэт. 2016 у.
 *      Author: vetal
 */

#include "command.h"
#include "string.h"
#include "stdlib.h"

static TaskHandle_t hCommandTask;

static void processPingCommand(UART_HandleTypeDef *hUART, uint8_t *command) {
	hCommandTask = xTaskGetCurrentTaskHandle();

	HAL_UART_StateTypeDef state = HAL_UART_GetState(hUART);

	uint8_t *pingArg = command + CMD_PING_LENGTH;

	if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX) {
		HAL_UART_Transmit_DMA(hUART, pingArg, strlen((char*)pingArg));

		vTaskSuspend(hCommandTask);

		return;
	}
}

static void writePinCommand(uint8_t *pinStr, GPIO_PinState state) {

	int pin = atoi((char*) pinStr);

	if (isPinOK(pin)) {
		HAL_GPIO_WritePin(GPIOD, 1 << pin, state);
	}
}

void executeCommand(UART_HandleTypeDef *hUART, uint8_t *command) {
	//size_t strLenght = strlen((char*)command);

	if (strncmp((char*)command, CMD_PING, CMD_PING_LENGTH) == 0) {
		processPingCommand(hUART, command);
	} else if (strncmp((char*)command, CMD_ON, CMD_ON_LENGTH) == 0) {
		writePinCommand(command + CMD_ON_LENGTH, GPIO_PIN_SET);
	} else if (strncmp((char*)command, CMD_OFF, CMD_OFF_LENGTH) == 0) {
		writePinCommand(command + CMD_OFF_LENGTH, GPIO_PIN_RESET);
	}


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	xTaskResumeFromISR(hCommandTask);
}

