/*
 * functions.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void NVIC_EnableInterrupts(FunctionalState newState);
void extPeripheralInit(void);
void Delaynus(vu32 nus);
void transferDMA_USART2(uint8_t *data, int length);
void transferDMA_USART3(uint8_t *data, int length);

#endif /* FUNCTIONS_H_ */
