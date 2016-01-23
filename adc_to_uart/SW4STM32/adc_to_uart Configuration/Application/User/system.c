#include "system.h"
#include <stdarg.h>

void processError(uint8_t nArgs, ...) {
	va_list ar;

	va_start(ar, nArgs);

	while (1) {
		for (uint8_t i = 0; i < nArgs; i++) {
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, i % 2 == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

			HAL_Delay(va_arg(ar, int));
		}
	}
}
