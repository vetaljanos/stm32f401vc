/*
 * l2c.h
 *
 *  Created on: 23 џэт. 2016 у.
 *      Author: vetal
 */

#ifndef APPLICATION_USER_L2C_H_
#define APPLICATION_USER_L2C_H_

#include "stm32f4xx_hal.h"
#include "system.h"
//#include "lsm303dlhc_driver.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f401_discovery_accelerometer.h"

#define MODE_1 //rotate device and led's will on/off
//#define MODE_2	//send axis data to UART

#define I2C_TIMEOUT 1000
#define AXIS_THRESHOLD 400
#define AXIS_HYSTERESIS 100

#define i2cError() processError(4, 1000, 1000, 500, 500)

void startAndCheckAccelerometr();
void update_LEDS_ByAccelerometr();

#endif /* APPLICATION_USER_L2C_H_ */
