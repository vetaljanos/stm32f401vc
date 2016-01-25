#include "uart.h"
#include "system.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart2;

void UART_Send(uint8_t *buffer, uint8_t length) {
	if (HAL_UART_Transmit_DMA(&huart2, buffer, length) != HAL_OK) {
		processError(4, 100, 100, 100, 2000);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		free(huart2.pTxBuffPtr);
	}
}
