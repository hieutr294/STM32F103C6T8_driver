/*
 * i2c.h
 *
 *  Created on: Apr 17, 2024
 *      Author: minhh
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include <stdint.h>
#include "stm32f103xx.h"

typedef struct{

}I2C_Config_t;

typedef struct{
	I2C_Config_t* pI2CConfig;
	I2C_RegDef_t pI2Cx;
}I2C_Handle_t;

void I2C_ClockControl(I2C_RegDef_t* pI2Cx, uint8_t condition);
void I2C_Init(I2C_Handle_t* pI2CHandle);

#endif /* INC_I2C_H_ */
