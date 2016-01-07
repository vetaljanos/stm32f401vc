/*
 * main.h
 *
 *  Created on: 07 џэт. 2016 у.
 *      Author: vetal
 */

#ifndef APPLICATION_USER_MAIN_H_
#define APPLICATION_USER_MAIN_H_

#include "stm32f4xx_hal_gpio.h"

const uint16_t x_LEDS = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
const uint16_t x_LED[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

#endif /* APPLICATION_USER_MAIN_H_ */
