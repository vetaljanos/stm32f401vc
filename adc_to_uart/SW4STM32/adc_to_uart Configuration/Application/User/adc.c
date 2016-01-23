#include "adc.h"
#include "system.h"
#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;

static uint32_t adcBuffer[ADC_BUFFER_SIZE];
static uint32_t adcResults[3];
static uint32_t before;
static uint32_t after;

static void calculateAvgValue(ADC_HandleTypeDef *hadc, uint8_t position) {
	uint64_t values[] = {0, 0, 0};

	before = hadc->DMA_Handle->Instance->NDTR;

	for (int i = position; i < ADC_HALF_BUFFER_SIZE + position; i++) {
		values[i % 3] += adcBuffer[i];
	}

	for (int i = 0; i < 3; i++) {
		adcResults[i] = values[i] * 3 / ADC_HALF_BUFFER_SIZE;
	}

	after = hadc->DMA_Handle->Instance->NDTR;

	if (after > before) {
		processError(2, 500, 500);
	}
}

uint32_t getAdcValue(uint8_t channel) {
	return adcResults[channel];
}

void adc_start() {
	if (HAL_ADC_Start_DMA(&hadc1, adcBuffer, ADC_BUFFER_SIZE) != HAL_OK) {
		processError(2, 1000, 500);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->DMA_Handle->Instance == hdma_adc1.Instance && hadc->Instance == hadc1.Instance)
	{
		calculateAvgValue(hadc, ADC_HALF_BUFFER_SIZE);
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->DMA_Handle->Instance == hdma_adc1.Instance && hadc->Instance == hadc1.Instance)
	{
		calculateAvgValue(hadc, 0);
	}
}
