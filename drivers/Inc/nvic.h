/*
 * nvic.h
 *
 *  Created on: Apr 8, 2024
 *      Author: minhh
 */

#ifndef INC_NVIC_H_
#define INC_NVIC_H_

void nvicEnable(int IRQNumber);
void nvicSetPriority(uint8_t IRQNumber, uint8_t priority);

#endif /* INC_NVIC_H_ */
