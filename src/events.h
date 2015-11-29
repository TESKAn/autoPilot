/*
 * events.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef EVENTS_H_
#define EVENTS_H_

void ADC_ISR_Handler(void);
void TIM4_ISR_Handler(void);
void TIM8_CC_ISR_Handler(void);
void TIM1_BRK_TIM9_ISR_Handler(void);
void TIM8_TRG_COM_TIM14_ISR_Handler(void);
void DMA1_Stream6_ISR_Handler(void);
void DMA1_Stream4_ISR_Handler(void);
void DMA1_Stream3_ISR_Handler(void);
void DMA2_Stream7_ISR_Handler(void);
//void I2C2_EV_IRQHandler(void);
void TIM1_CC_ISR_Handler(void);
void USART1_ISR_Handler(void);
void USART2_ISR_Handler(void);
void USART3_ISR_Handler(void);
void EXTI0_ISR_Handler(void);
void FPU_ISR_Handler(void);

#endif /* EVENTS_H_ */
