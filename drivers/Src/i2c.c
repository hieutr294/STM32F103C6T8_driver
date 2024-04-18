/*
 * i2c.c
 *
 *  Created on: Apr 17, 2024
 *      Author: minhh
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "i2c.h"

void I2C_ClockControl(I2C_RegDef_t* pI2Cx, uint8_t condition){
	if(condition == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
	}else if(condition == DISABLE){
		if(pI2Cx==I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
	}
}

void I2C_Init(I2C_Handle_t* pI2CHandle){

	if(pI2CHandle->pI2CConfig.I2C_ACKControl==I2C_ACK_ENABLE){
		pI2CHandle->pI2Cx->CR1 |= (1<<CR1_ACK);
	}else if(pI2CHandle->pI2CConfig.I2C_ACKControl==I2C_ACK_DISABLE){
		pI2CHandle->pI2Cx->CR1 &= ~(1<<CR1_ACK);
	}

	if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_SM){

	    /* Flow the datasheet
	     * Thigh = CCR * Tpclk
	     * Tlow = CCR * Tpclk
	     * Because Thigh = Tlow
	     * => Tscl = Thigh + Tlow = 2*CCR*Tpclsk
	     * convert to frequency
	     * => CCR = FPCLK / (2*FSCL)
	     *  */

	    uint32_t fPCLK = 8000000;
		uint32_t ccrValue = fPCLK/(2*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

		pI2CHandle->pI2Cx->CCR &= ~(1<<CCR_FS);
		pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
		pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);

	}else if(pI2CHandle->pI2CConfig.I2C_SCLSpeed == I2C_SCL_SPEED_FM){
		pI2CHandle->pI2Cx->CCR |= (1<<CCR_FS);
		if(pI2CHandle->pI2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2){

		    /* Flow the datasheet FM mode and tlow/thigh = 2
		     * Thigh = CCR * Tpclk
		     * Tlow = 2*CCR * Tpclk
		     * Because Thigh = 2*Tlow
		     * => Tscl = Thigh + Tlow = 3*CCR*Tpclsk
		     * convert to frequency
		     * => CCR = FPCLK / (3*FSCL)
		     *  */

		    uint32_t fPCLK = 8000000;
			uint32_t ccrValue = fPCLK/(3*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

			pI2CHandle->pI2Cx->CCR &= ~(1<<CCR_DUTY);
			pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
			pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);

		}else if(pI2CHandle->pI2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_16_9){

		    /* Flow the datasheet FM mode and tlow/thigh = 16/9
		     * Thigh = 9*CCR * Tpclk
		     * Tlow = 16*CCR * Tpclk
		     * => Tscl = 9*Thigh + 16*Tlow = 25*CCR*Tpclsk
		     * convert to frequency
		     * => CCR = FPCLK / (25*FSCL)
		     *  */

		    uint32_t fPCLK = 8000000;
			uint32_t ccrValue = fPCLK/(25*pI2CHandle->pI2CConfig.I2C_SCLSpeed);

			pI2CHandle->pI2Cx->CCR |= (1<<CCR_DUTY);
			pI2CHandle->pI2Cx->CR2 |= (8<<CR2_FREQ);
			pI2CHandle->pI2Cx->CCR |= (ccrValue<<CCR_CCR);
		}

	}

	pI2CHandle->pI2Cx->OAR1 |= ((pI2CHandle->pI2CConfig.I2C_DeviceAddress<<1)<<OAR1_ADD);
	pI2CHandle->pI2Cx->OAR1 |= (1<<14); // bit 14 always set by software (datasheet)
}
