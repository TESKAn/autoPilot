/*
 * var.c
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#include "allinclude.h"

// Flag variable
Flag flag0;

// Temp variable
int globalTemp = 0;

// SCR reg
// Defined in modbus.h

//TIM1 variables
uint16_t TIM1_CCRValue4 = 2200;

uint16_t TIM1_changeDelay = 0;

int TIM1CaptureValue1 = 0;
int TIM1CaptureValue2 = 0;
int TIM1CaptureValue3 = 0;
int TIM1CaptureValue4 = 0;

// TIM4 variables
uint32_t TIM4_IC1_PreviousValue = 0;
uint32_t TIM4_IC1_LowWidth = 0;
uint32_t TIM4_IC1_HighWidth = 0;
uint32_t TIM4_IC2_PreviousValue = 0;
uint32_t TIM4_IC2_LowWidth = 0;
uint32_t TIM4_IC2_HighWidth = 0;
uint32_t TIM4_IC3_PreviousValue = 0;
uint32_t TIM4_IC3_LowWidth = 0;
uint32_t TIM4_IC3_HighWidth = 0;
uint32_t TIM4_IC4_PreviousValue = 0;
uint32_t TIM4_IC4_LowWidth = 0;
uint32_t TIM4_IC4_HighWidth = 0;

// TIM8 variables
uint32_t TIM8_IC1_PreviousValue = 0;
uint32_t TIM8_IC1_LowWidth = 0;
uint32_t TIM8_IC1_HighWidth = 0;
uint32_t TIM8_IC2_PreviousValue = 0;
uint32_t TIM8_IC2_LowWidth = 0;
uint32_t TIM8_IC2_HighWidth = 0;
uint32_t TIM8_IC3_PreviousValue = 0;
uint32_t TIM8_IC3_LowWidth = 0;
uint32_t TIM8_IC3_HighWidth = 0;
uint32_t TIM8_IC4_PreviousValue = 0;
uint32_t TIM8_IC4_LowWidth = 0;
uint32_t TIM8_IC4_HighWidth = 0;

// GPS variables
// Defined in modbus.h

// Power sensor variables
// Defined in modbus.h

// ADC trigger timer
uint32_t ADC_TriggerTimer = 0;

//DMA variables
volatile uint8_t UART2DMAbuffer[20];
