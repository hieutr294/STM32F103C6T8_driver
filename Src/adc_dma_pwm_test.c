/*
 * adc_dma_pwm_test.c
 *
 *  Created on: Apr 9, 2024
 *      Author: minhh
 */


/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <pwm.h>
#include <stdint.h>
#include "stm32f103xx.h"
#include "gpio.h"
#include "adc.h"
#include "dma.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

uint8_t adcChannelArray[1] = {1};
volatile uint8_t dmaData = 0;

int main(void)
{

	DMA_Handle_t dma1;
	ADC_Handle_t adc1;

	adc1.pADCX = ADC1;
	adc1.pADCConfig.channel = adcChannelArray;
	adc1.pADCConfig.channelGroup = REGULAR_MODE;
	adc1.pADCConfig.conversionMode = SCAN_MODE;
	adc1.pADCConfig.numberChannel = 1;
	adc1.pADCConfig.sampleTime = CYCLE_1;
	adc1.pADCConfig.dmaEnable = ENABLE;

	dma1.pDMAX = DMA1;
	dma1.pDMAConfig.channel = 1;
	dma1.pDMAConfig.channelPriority = HIGH_PRIORITY;
	dma1.pDMAConfig.circularMode = CIRC_ENA;
	dma1.pDMAConfig.dataDirection = READ_FROM_PERI;
	dma1.pDMAConfig.memIncrement = MINC_DIS;
	dma1.pDMAConfig.peripheralIncrement = PINC_DIS;
	dma1.pDMAConfig.mode = PERI2MEM;
	dma1.pDMAConfig.byteToTransfer = 1;

	ADC_ClockControl(ADC1,ENABLE);

	DMA_ClockControl(DMA1,ENABLE);

	ADC_Init(&adc1);
	ADC_Start(&adc1);

	DMA_Init(&dma1);

	DMA_SetMemoryAddress(&dma1, MEMSIZE_16, &dmaData);

	DMA_SetPeripheralAddress(&dma1, PSIZE_16, &ADC1->DR);

	DMA_EnableChannel(&dma1);

	PWM_Handle_t pwm1;

	pwm1.pTimerX = TIM1;
	pwm1.pTimerConfig.channel = 1;
	pwm1.pTimerConfig.counter = COUNTER_UP;
	pwm1.pTimerConfig.mode = PWM_MODE_1;
	pwm1.pTimerConfig.alighMode = EDGE_ALIGN;
	pwm1.pTimerConfig.polarity = 1;
	pwm1.pTimerConfig.prescale = 8;
	pwm1.pTimerConfig.frequency = 1000;

	Timer_ClockControl(TIM1,ENABLE);
	PWM_Init(&pwm1);

	while(1){

	}
}

//int convert(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
