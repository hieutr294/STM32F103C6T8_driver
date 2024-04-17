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


