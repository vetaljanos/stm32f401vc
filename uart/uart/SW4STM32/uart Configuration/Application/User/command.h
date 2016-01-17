/*
 * command.h
 *
 *  Created on: 17 џэт. 2016 у.
 *      Author: vetal
 */

#ifndef APPLICATION_USER_COMMAND_H_
#define APPLICATION_USER_COMMAND_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define CMD_PING "ping "
#define CMD_PING_LENGTH ((size_t) 5)

#define CMD_ON "on "
#define CMD_ON_LENGTH ((size_t) 3)

#define CMD_OFF "off "
#define CMD_OFF_LENGTH ((size_t) 4)

#define isPinOK(pin) pin >= 0 && pin < 16

void executeCommand(UART_HandleTypeDef *hUART, uint8_t *command);

#endif /* APPLICATION_USER_COMMAND_H_ */
