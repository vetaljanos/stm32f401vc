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
static Environment *environment;
//static uint16_t *TS_CAL1 = 0x1FFF7A2C;
//static uint16_t *TS_CAL2 = 0x1FFF7A2E;

static uint32_t adc0Buffer[100];

static void processPingCommand(uint8_t *command) {
	hCommandTask = xTaskGetCurrentTaskHandle();

	HAL_UART_StateTypeDef state = HAL_UART_GetState(environment->uartHandler);

	uint8_t *pingArg = command + CMD_PING_LENGTH;

	size_t length = strlen((char*) pingArg);

	if (length > 0) {
		//check for \n at the end of string

		char lastChar = pingArg[length-1];

		uint8_t strWasModified = 0;

		if (lastChar != '\n') {
			char *newStr = malloc(length+2);
			strcpy(newStr, (char *)pingArg);
			strcat(newStr, "\n");
			pingArg = (uint8_t *)newStr;
			length++;

			strWasModified = 1;
		}

		int waitIteration = 0;

		while (1) {
			if (state == HAL_UART_STATE_READY || state == HAL_UART_STATE_BUSY_RX) {
				HAL_UART_Transmit_DMA(environment->uartHandler, pingArg, length);

				vTaskSuspend(hCommandTask);

				if (strWasModified) {
					free(pingArg);
				}

				return;
			}

			if (waitIteration > 1000) {
				break;
			}

			waitIteration++;
		}
	}
}

static void writePinCommand(uint8_t *pinStr, GPIO_PinState state) {

	int pin = atoi((char*) pinStr);

	if (isPinOK(pin)) {
		HAL_GPIO_WritePin(GPIOD, 1 << pin, state);
	}
}

static void processADCCommand(uint8_t *args) {
//	if (environment->adcHandler->Instance->CR2 == ADC_CR2_DMA) {
		//dma is working. Stop it

	//	HAL_ADC_Stop_DMA(environment->adcHandler);
	//} else {
		HAL_ADC_Start_DMA(environment->adcHandler, adc0Buffer, 100);
	//}
}

void executeCommand(Environment *env, uint8_t *command) {
	//size_t strLenght = strlen((char*)command);

	environment = env;

	if (strncmp((char*) command, CMD_PING, CMD_PING_LENGTH) == 0) {
		processPingCommand(command);
	} else if (strncmp((char*) command, CMD_ON, CMD_ON_LENGTH) == 0) {
		writePinCommand(command + CMD_ON_LENGTH, GPIO_PIN_SET);
	} else if (strncmp((char*) command, CMD_OFF, CMD_OFF_LENGTH) == 0) {
		writePinCommand(command + CMD_OFF_LENGTH, GPIO_PIN_RESET);
	} else if (strncmp((char*) command, CMD_ADC, CMD_ADC_LENGTH) == 0) {
		processADCCommand(command + CMD_ADC_LENGTH);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	xTaskResumeFromISR(hCommandTask);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	char *str = malloc(25);

	sprintf(str, "ping channel = %08ld\n", adc0Buffer[99]);

	//push it to the same queue which process UART input
	xQueueSendFromISR(environment->commandQueueHandler, &str, 0);

}
