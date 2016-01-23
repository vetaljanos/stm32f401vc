#ifndef __ADC__X
#define __ADC__X

#include "stm32f4xx_hal.h"

#define ADC_BUFFER_SIZE ((uint8_t)120)
#define ADC_HALF_BUFFER_SIZE ((uint8_t) ADC_BUFFER_SIZE / 2)

uint32_t getAdcValue(uint8_t ADC_Channel);

void adc_start();

#endif
