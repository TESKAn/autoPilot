/*
 * init.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef INIT_H_
#define INIT_H_

void init_Timer1();
void init_Timer2();
void init_Timer3();
void init_Timer4();
void init_Timer6();
void init_Timer8();
void init_Timer9();
void init_Timer14();
void init_ADC();
void init_DAC();
void init_USART1();
void init_USART2();
void init_USART3();
void init_SPI();
void init_GPIO();
void NVIC_EnableInterrupts(FunctionalState newState);
void System_Config(void);

#endif /* INIT_H_ */
