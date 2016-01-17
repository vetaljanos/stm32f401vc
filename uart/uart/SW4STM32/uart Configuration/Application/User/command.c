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

static void processADCTemperature() {
	ADC_ChannelConfTypeDef config;

	config.Channel = ADC_CHANNEL_TEMPSENSOR;
	config.Rank = 1;
	config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(environment->adcHandler, &config);

	HAL_ADC_Start_IT(environment->adcHandler);
}

static void processADCCommand(uint8_t *args) {
	//int pin = atoi((char*) pinStr);
	char *pinStr = strtok((char*) args, " ");
	char *samplesStr = strtok(NULL, " ");

	int pin = atoi(pinStr);

	if (pin >= 0 && pin <= 18) {
		//not defined

		uint32_t channel;

		switch (pin) {
		case 0:channel = ADC_CHANNEL_0;break;
		case 16:channel = ADC_CHANNEL_16;break;
		default:channel = ADC_CHANNEL_0;
			//define another channels if you need
		}

		uint32_t samplingTime;

		int samples = samplesStr == NULL ? 1 : atoi(samplesStr);

		switch (samples) {
		case 1:samplingTime = ADC_SAMPLETIME_3CYCLES;break;
		case 2:samplingTime = ADC_SAMPLETIME_84CYCLES;break;
		case 3:samplingTime = ADC_SAMPLETIME_480CYCLES;break;
		default: samplingTime = ADC_SAMPLETIME_480CYCLES;break;
		}

		ADC_ChannelConfTypeDef config;

		config.Channel = channel;
		config.Rank = 1;
		config.SamplingTime = samplingTime;
		HAL_ADC_ConfigChannel(environment->adcHandler, &config);

		HAL_ADC_Start_IT(environment->adcHandler);
	}
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
	} else if (strncmp((char*) command, CMD_TEMP, CMD_TEMP_LENGTH) == 0) {
		processADCTemperature();
	} else if (strncmp((char*) command, CMD_ADC, CMD_ADC_LENGTH) == 0) {
		processADCCommand(command + CMD_ADC_LENGTH);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	xTaskResumeFromISR(hCommandTask);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	uint32_t value = HAL_ADC_GetValue(AdcHandle);

	char *str = malloc(34);

	sprintf(str, "ping channel %08ld = %08ld\n", AdcHandle->Instance->SQR3, value);

	//push it to the same queue which process UART input
	xQueueSendFromISR(environment->commandQueueHandler, &str, 0);
}
