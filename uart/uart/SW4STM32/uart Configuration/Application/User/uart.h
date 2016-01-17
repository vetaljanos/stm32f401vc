/*
 * uart.h
 *
 *  Created on: 17 џэт. 2016 у.
 *      Author: vetal
 */

#ifndef APPLICATION_USER_UART_H_
#define APPLICATION_USER_UART_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define MAX_STR_LENGTH ((uint8_t) 255 - 1)

void startUART_DMA_Receiver(UART_HandleTypeDef *hUART, osMessageQId *cmdQueue);

#endif /* APPLICATION_USER_UART_H_ */
