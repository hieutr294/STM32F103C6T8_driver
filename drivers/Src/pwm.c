/*
 * timer.c
 *
 *  Created on: Mar 25, 2024
 *      Author: minhh
 */

#include <pwm.h>
#include <stdint.h>
#include "stm32f103xx.h"
#include "gpio.h"

void Timer_ClockControl(TIMER_RegDef_t* pTimerX, uint8_t condition){
	if(condition==ENABLE){
		if(pTimerX==TIM1){
			TIM1_PCLK_EN();
		}
		else if(pTimerX==TIM8){
			TIM8_PCLK_EN();
		}
	}
	else if(condition==DISABLE){
		if(pTimerX==TIM1){
			TIM1_PCLK_DI();
		}
		else if(pTimerX==TIM8){
			TIM8_PCLK_DI();
		}
	}
}

void PWM_Init(PWM_Handle_t* pTimerHandle){
	GPIO_Handle_t gpio;
	uint32_t timerClock = 0;
	uint32_t arrValue = 0;

	gpio.GPIO_PinConfig.pinNumber = pTimerHandle->pTimerConfig.channel+7;
	gpio.GPIO_PinConfig.pinMode = AF_PUSH_PULL_50MHZ;
	gpio.pGPIOX = GPIOA;

	GPIO_ClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpio);

	switch(pTimerHandle->pTimerConfig.channel){
		case 1:
			pTimerHandle->pTimerX->CCER |= (1<<CC1E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC1P_POS);
			pTimerHandle->pTimerX->CCMR1 |= (pTimerHandle->pTimerConfig.mode<<OC1M_POS);
			break;
		case 2:
			pTimerHandle->pTimerX->CCMR1 |= (pTimerHandle->pTimerConfig.mode<<OC2M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC2E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC2P_POS);
			break;
		case 3:
			pTimerHandle->pTimerX->CCMR2 |= (pTimerHandle->pTimerConfig.mode<<OC3M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC3E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC3P_POS);
			break;
		case 4:
			pTimerHandle->pTimerX->CCMR2 |= (pTimerHandle->pTimerConfig.mode<<OC4M_POS);
			pTimerHandle->pTimerX->CCER |= (1<<CC4E_POS);
			pTimerHandle->pTimerX->CCER |= (pTimerHandle->pTimerConfig.polarity<<CC4P_POS);
			break;
	}

	if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_1){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_1<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_2){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_2<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == CENTER_ALIGH_MODE_3){
		pTimerHandle->pTimerX->CR1 |= (CENTER_ALIGH_MODE_3<<CMS_POS);
	}
	else if(pTimerHandle->pTimerConfig.alighMode == EDGE_ALIGN){
		pTimerHandle->pTimerX->CR1 &= ~(CENTER_ALIGH_MODE_3<<CMS_POS);
	}

	if(pTimerHandle->pTimerConfig.counter == COUNTER_UP){
		pTimerHandle->pTimerX->CR1 &= ~(1<<DIR_POS);
	}
	else if(pTimerHandle->pTimerConfig.counter == COUNTER_DOWN){
		pTimerHandle->pTimerX->CR1 |= (1<<DIR_POS);
	}
	if(pTimerHandle->pTimerConfig.preload==PRELOAD_ENA){
		pTimerHandle->pTimerX->CR1 |= (1<<ARPE_POS);
	}
	else if(pTimerHandle->pTimerConfig.preload==PRELOAD_DIS){
		pTimerHandle->pTimerX->CR1 &= ~(1<<ARPE_POS);
	}

	timerClock = 8000000/pTimerHandle->pTimerConfig.prescale;
	arrValue = timerClock/pTimerHandle->pTimerConfig.frequency;

	pTimerHandle->pTimerX->PSC = pTimerHandle->pTimerConfig.prescale-1;
	pTimerHandle->pTimerX->ARR = arrValue-1;

	pTimerHandle->pTimerX->BDTR |= (1<<MOE_POS);

	pTimerHandle->pTimerX->CR1 |= (1<<CEN_POS);
}

void PWM_SetDuty(PWM_Handle_t* pTimerHandle, uint8_t duty){
	switch(pTimerHandle->pTimerConfig.channel){
		case 1:
			pTimerHandle->pTimerX->CCR1 = (duty*((uint32_t)(pTimerHandle->pTimerX->ARR)+1))/100;
			break;
		case 2:
			pTimerHandle->pTimerX->CCR2 = (duty*((uint32_t)(pTimerHandle->pTimerX->ARR)+1))/100;
			break;
		case 3:
			pTimerHandle->pTimerX->CCR3 = (duty*((uint32_t)(pTimerHandle->pTimerX->ARR)+1))/100;
			break;
		case 4:
			pTimerHandle->pTimerX->CCR4 = (duty*((uint32_t)(pTimerHandle->pTimerX->ARR)+1))/100;
			break;
	}
}
