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

#include <stdint.h>
#include "stm32f103xx.h"
#include "i2c.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


int main(void){
	I2C_Handle_t i2c1;
	i2c1.pI2Cx = I2C1;
	i2c1.pI2CConfig.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_ClockControl(I2C1, ENABLE);
	I2C_Init(&i2c1);

	return 0;
};
