/*
 * events.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef EVENTS_H_
#define EVENTS_H_

void ADC_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);


#endif /* EVENTS_H_ */
