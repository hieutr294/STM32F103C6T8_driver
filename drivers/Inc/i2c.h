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

#define I2C_SCL_SPEED_SM			100000 // Standard mode
#define I2C_SCL_SPEED_FM			400000 // Fast mode

#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE 			0

#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

#define CR2_FREQ					0
#define CCR_CCR						0
#define CCR_DUTY					14
#define CR1_ACK						10
#define OAR1_ADD					1
#define CCR_FS						15

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_Config_t pI2CConfig;
	I2C_RegDef_t* pI2Cx;
}I2C_Handle_t;

void I2C_ClockControl(I2C_RegDef_t* pI2Cx, uint8_t condition);
void I2C_Init(I2C_Handle_t* pI2CHandle);

#endif /* INC_I2C_H_ */
