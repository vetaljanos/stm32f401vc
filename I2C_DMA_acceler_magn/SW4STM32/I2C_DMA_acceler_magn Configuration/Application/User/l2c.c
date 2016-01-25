/*
 * l2c.c
 *
 *  Created on: 23 џэт. 2016 у.
 *      Author: vetal
 */

#include "l2c.h"

extern I2C_HandleTypeDef hi2c1;

void startAndCheckAccelerometr() {
	//wait for I2C bus

	uint32_t errorThreshold = HAL_GetTick() + 100;

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		if (errorThreshold > HAL_GetTick()) {
			i2cError();
		}
	}

	//check if device is ready

	if (HAL_I2C_IsDeviceReady(&hi2c1, ACC_I2C_ADDRESS, 10, 100) != HAL_OK) {
		i2cError();
	}

	if (BSP_ACCELERO_Init() != HAL_OK) {
		i2cError();
	}
}

void update_LEDS_ByAccelerometr() {
	int16_t buffer[3] = { 0 };

	int16_t x, y, z = 0x00;

	BSP_ACCELERO_GetXYZ(buffer);

	x = buffer[0];
	y = buffer[1];
	z = buffer[2];

	if (z > 0) {
		if (x > AXIS_THRESHOLD) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
		} else if (x < AXIS_THRESHOLD - AXIS_HYSTERESIS) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
		}

		if (x < -AXIS_THRESHOLD) {
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin,
					GPIO_PIN_SET);
		} else if (x > -AXIS_THRESHOLD + AXIS_HYSTERESIS) {
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin,
					GPIO_PIN_RESET);
		}

		if (y > AXIS_THRESHOLD) {
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		} else if (y < AXIS_THRESHOLD - AXIS_HYSTERESIS) {
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		}

		if (y < -AXIS_THRESHOLD) {
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
		} else if (y > -AXIS_THRESHOLD + AXIS_HYSTERESIS) {
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
					GPIO_PIN_RESET);
		}
	} else {
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	}
}

