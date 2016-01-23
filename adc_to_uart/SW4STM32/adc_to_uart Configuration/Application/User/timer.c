#include "timer.h"
#include "adc.h"
#include "system.h"
#include "uart.h"
#include "stdlib.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint8_t nextChannel = 0;

void startTimer1() {
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		processError(4, 500, 2000, 500, 500);
	}
}

void startTimer2() {
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		processError(6, 500, 500, 500, 2000, 500, 500);
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance)
	{
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	}
	else if (htim->Instance == htim2.Instance) {
		char *buffer = malloc(sizeof(char) * 16);

		uint32_t value = getAdcValue(nextChannel);

		int l = sprintf(buffer, "%d: %08ld\n", nextChannel, value);

		UART_Send((uint8_t *)buffer, l);

		nextChannel++;

		if (nextChannel == 3) {
			nextChannel = 0;
		}
	}
}
