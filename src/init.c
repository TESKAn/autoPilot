/*
 * init.c
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#include "allinclude.h"

// Init timer 1

void init_Timer1()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

	// Init GPIO
	//config GPIOE pins 9,11,13,14 for output compare
	//set AF pin source to TIM1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	//configure structure
	//select pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	// Timer 1
	// Enable clock(s)
	// Clock = 84 MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM1_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM1_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 1
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = TIM1_PULSE;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	// Configure output compare, channel 1
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break= TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);

	// Required for timers 1 or 8
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// Enable interrupts
	// TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	// Enable timer
	TIM_Cmd(TIM1, ENABLE);
	// End of Timer 1
}

void init_Timer2()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

	// Configure GPIO
	// A0 - A3, A6, A7 timer output
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	// Timer 2
	// Enable clock(s)
	// Clock = 42 MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM2_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM2_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 2
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = TIM2_PULSE;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	// Configure output compare, channel 2
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM2, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break= TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM2, &TIM_BDTRInitStruct);

	// Required for timers 1 or 8
	//TIM_CtrlPWMOutputs(TIM2, ENABLE);

	// Enable interrupts
	// TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	// Enable timer
	TIM_Cmd(TIM2, ENABLE);
	// End of Timer 2
}

void init_Timer3()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

	// Configure GPIO
	//connect pins B0 and B1 to timer output
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	//select pins 0 and 1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure GPIO
	// A0 - A3, A6, A7 timer output

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Timer 3
	// Enable clock(s)
	// Clock = 42 MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM3_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM3_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 3
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = TIM3_PULSE;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	// Configure output compare, channel 3
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM3, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break= TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM3, &TIM_BDTRInitStruct);

	// Required for timers 1 or 8
	//TIM_CtrlPWMOutputs(TIM2, ENABLE);

	// Enable interrupts
	// TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	// Enable timer
	TIM_Cmd(TIM3, ENABLE);
	// End of Timer 3
}

void init_Timer4()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	// Configure GPIO
	//connect pins D12, D13, D14, D15 to timer input
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	//select pins 12 - 15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Timer 4 - input capture
	// Enable clock(s)
	// Clock = 42 MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM4_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM4_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 4
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = TIM4_FILTER;
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM4, &TIM_ICInitStruct);

	// Configure interrupt
	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	// Enable timer
	TIM_Cmd(TIM4, ENABLE);

	// End of timer 4
}

void init_Timer6()
{
	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	// Timer 6 - timing for DAC

	// TIM6 Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	// Time base configuration
	//TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_Period = 0xFF;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);

	// TIM6 TRGO selection
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	// TIM6 enable counter
	TIM_Cmd(TIM6, ENABLE);

	// End of timer 6
}

void init_Timer8()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	// Configure GPIO
	//connect pins C6, C7, C8, C9 to timer input
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
	//select pins 0 and 1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Timer 8 - input capture
	// Enable clock(s)
	// Clock = 84 MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM8_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM8_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 8
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = TIM8_FILTER;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);

	// Configure interrupt
	TIM_ITConfig(TIM8, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	// Enable timer
	TIM_Cmd(TIM8, ENABLE);

	// End of timer 8
}

void init_Timer9()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;

	// Configure GPIO
	//connect pin E5 to timer input
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	//select pin 5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Timer 9 - lost packet indicator
	// Enable clock(s)
	// Clock = 84 MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	// Populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM9_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM9_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	// Configure timer 9
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseInitStruct);

	// Populate structure
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = TIM9_FILTER;
	TIM_ICInit(TIM9, &TIM_ICInitStruct);


	// Configure interrupt
	TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);

	// Enable timer
	TIM_Cmd(TIM9, ENABLE);

	// End timer 9
}

void init_Timer14()
{
	//make structures for configuring timers
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	// Timer 14 - timing
	//enable clock(s) - 42 MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	//populate structure
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;	//1 - 4
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM14_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM14_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	//configure timer 14
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct);
	//enable interrupt
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	// Enable reload
	//TIM_ARRPreloadConfig(TIM14, ENABLE);
	//enable timer
	TIM_Cmd(TIM14, ENABLE);
	// End of timer 14
}

void init_ADC()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Init GPIO
	//connect pins C0, C1, C2, C3 to ADC input
	//select pins 0, 1, 2, 3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// ADC converter init
	// Enable clock(s) (84 MHz)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	// ADC Common configuration
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Configure
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 2;

	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_Init(ADC2, &ADC_InitStruct);
	ADC_Init(ADC3, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

	// Configure interrupt
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);
	// Set interrupt after all channels are scanned
	//ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	// Enable ADC
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	// End of ADC
}

void init_DAC(void)
{
	DAC_InitTypeDef DAC_InitStruct;
	DMA_InitTypeDef DMAInitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	// init DAC pins
	// Configure GPIO pins
	/* GPIO Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//GPIO A
	// A4, A5 - DAC output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// DMA for DAC
	// DMA1_Stream5 channel7 configuration
	DMA_DeInit(DMA_DAC1);
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R1_ADDRESS;
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)&Sine12bit;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = 32;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMAInitStructure.DMA_Mode = DMA_Mode_Circular;
	DMAInitStructure.DMA_Priority = DMA_Priority_High;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA_DAC1, &DMAInitStructure);

	// Enable DMA1_Stream5
	DMA_Cmd(DMA_DAC1, ENABLE);

	// DAC init
	// Enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	// Setup channel 1
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	// Enable DAC Channel1
	DAC_Cmd(DAC_Channel_1, ENABLE);
	// Enable DMA for DAC Channel1
	DAC_DMACmd(DAC_Channel_1, ENABLE);
	// Setup channel 2
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;//DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_2, &DAC_InitStruct);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	// End of DAC
}

void init_USART1()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structure for configuring USART
	USART_InitTypeDef USART_InitStructure;

	// Init GPIO
	//connect pins B6 and B7 to USART
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	//select pins 6 and 7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//configure module 1 - current/voltage/temperature sensor
	//set baud rate
	USART_InitStructure.USART_BaudRate = 19200;
	//flow control
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//enable receiver and transmitter
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//parity
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//word length
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//start port
	USART_Init(USART1, &USART_InitStructure);

	//enable interrupt - RX not empty, transfer complete
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//enable module 1
	USART_Cmd(USART1, ENABLE);
}

void init_USART2()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	//make structure for configuring USART
	USART_InitTypeDef USART_InitStructure;

	// Config GPIO
	//connect pins D5 and D6 to USART2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	//select pins 5 and 6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// Configure USART2
	//Remember to set GPIO pins in GPIO configuration
	//enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //for USART1 and USART6
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE); //for USART2, USART3, UART4 or UART5.
	//program port parameters
	//set baud rate
	USART_InitStructure.USART_BaudRate = 38400;
	//flow control
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//enable receiver and transmitter
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//parity
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//word length
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//start port
	USART_Init(USART2, &USART_InitStructure);

	//enable interrupt - RX not empty, transfer complete
	//USART_ITConfig(USART2, USART_IT_RXNE | USART_IT_TC, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);


	//enable module 2
	USART_Cmd(USART2, ENABLE);

	// Configure USART2 DMA
	//enable peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//configure USART2 DMA channel
	//deinit DMA channel
	DMA_DeInit(DMA_USART2);
	//enable interrupts if needed
	//DMA_IT_TC - transfer complete interrupt
	//DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}

void init_USART3(void)
{
	//make structure for configuring USART
	USART_InitTypeDef USART_InitStructure;
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Init GPIO
	//connect pins C10 and C11 to USART
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	//select pins 10 and 11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// USART 3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//connect pins C10 and C11 to USART
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	//select pins 10 and 11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//configure module 3 - GPS
	//set baud rate
	USART_InitStructure.USART_BaudRate = 9600;
	//flow control
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//enable receiver and transmitter
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//parity
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//stop bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//word length
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//start port
	USART_Init(USART3, &USART_InitStructure);

	//enable interrupt - RX not empty, transfer complete
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	//enable module 3
	USART_Cmd(USART3, ENABLE);

	// Configure USART3 DMA
	//enable peripheral clock
	// Already enabled for USART2
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//configure USART3 DMA channel
	//deinit DMA channel
	DMA_DeInit(DMA_USART3);
	//enable interrupts if needed
	//DMA_IT_TC - transfer complete interrupt
	//DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}

// External interrupt config
void init_EXTI()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Set C5 as input, MPU6000 interrupt
	// Select C5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

	EXTI_Init(&EXTI_InitStructure);
}

// Only init SPI pins, rest is in FATFS functions
void init_SPI()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Connect B13, B14, B15 to SPI for SD card
	// Select AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	// Select pins 13, 14, 15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	// Set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// Push pull output
	// Set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// Set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// Set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	// Write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void init_GPIO()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;
	// Configure GPIO pins
	/* GPIO Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//GPIO A
	// A8 = GPIO output, set to 1, for SD card power
	// Set pin to 1
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1);
	// Select pin 8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to out function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Set pin to 1
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1);

	//GPIO B

	// B4 = GPIO input for SD card detect
	// Select pin 4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to out function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// B12 = GPIO output, set to 1, for SD card SS
	// Set pin to 1
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 1);
	// Select pin 12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to out function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Set pin to 1
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 1);

	//GPIO C

	// Set C12 as input, GPS PPS pulse
	// Select C12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Set C4 as input, sensor FSYNC
	// Select C4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Set pin to 0
	GPIO_WriteBit(GPIOC, GPIO_Pin_4, 0);

	// Set C5 as input, MPU6000 interrupt
	// Select C5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Set interrupt

	// Set C13 as output, pull to zero
	// C13 = USB OTG power enable
	// Set pin to 1
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, 1);
	// Select C13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Set pin to 1 - disable switch
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, 1);

	// Set C15 as output, set to 1, pull - up, MPL sensor shutdown
	// Select C15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Set pin to 1
	GPIO_WriteBit(GPIOC, GPIO_Pin_15, 1);

	//GPIO D

	// D3 = input, USB OTG power fault
	//configure structure
	//select pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Pins 8, 9, 10 as output low - status LEDs
	//select pins 8,9,10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// Set pins to 0
	GPIO_WriteBit(GPIOD, GPIO_Pin_8, 0);
	GPIO_WriteBit(GPIOD, GPIO_Pin_9, 0);
	GPIO_WriteBit(GPIOD, GPIO_Pin_10, 0);

	// Pin D 0 - ext. slave select 1
	//select pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//set pin mode
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//GPIO E

	// E7 = GPIO output, set to 1, for sensor power enable
	// Set pin to 1
	GPIO_WriteBit(GPIOE, GPIO_Pin_7, 1);
	// Select pin 8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to out function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	// Set pin to 1
	GPIO_WriteBit(GPIOE, GPIO_Pin_7, 1);

	// Connect pin E15 to output, PWM enable output
	// This pin switches MUX input select circuit
	//select pin 15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	// push/pull
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void NVIC_EnableInterrupts(FunctionalState newState)
{
	//interrupt controller
	NVIC_InitTypeDef NVCInitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//init ADC interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = ADC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART1 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART1_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART2_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream6 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM8 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM8 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM14 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init I2C2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	// Init EXTI interrupt
	// Enable and set EXTI Line0 Interrupt to the lowest priority
	NVCInitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 0x15;
	NVCInitStructure.NVIC_IRQChannelSubPriority = 0x15;
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init FPU interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = FPU_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	if(newState)
	{
		SYSTEM_INTERRUPTS_ON = 1;
	}
	else
	{
		SYSTEM_INTERRUPTS_ON = 0;
	}
}

// Init pin C0 ad GPIO input with pull - down
void init_swin()
{
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Init GPIO
	//connect pins C0 to GPIO
	//select pin 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// Open drain
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	//set pin mode to GPIO input
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void System_Config(void)
{

	init_GPIO();

	// Configure all the timers

	init_Timer1();
	init_Timer2();
	init_Timer3();
	init_Timer4();
	init_Timer6();
	init_Timer8();
	init_Timer9();
	init_Timer14();
	// USART ports
	init_USART1();
	init_USART2();
	init_USART3();
	// External interrupt
	init_EXTI();
	// SPI config
	init_SPI();
	// I2C2 config
	I2C2_Configure(ENABLE);
	// ADC config
	init_ADC();
	// DAC config
	init_DAC();
	// Enable interrupts - don't forget to enable specific interrupts
	NVIC_EnableInterrupts(ENABLE);
	// Init ADC0 as GPIO with pull - down
	init_swin();
}

