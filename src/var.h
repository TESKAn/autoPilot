/*
 * var.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef VAR_H_
#define VAR_H_

// Flag register typedef
typedef union
{
	struct
	{
		uint32_t flag;
	}flag;

	 struct
	 {
		 uint8_t BIT0:1;
		 uint8_t BIT1:1;
		 uint8_t BIT2:1;
		 uint8_t BIT3:1;
		 uint8_t BIT4:1;
		 uint8_t BIT5:1;
		 uint8_t BIT6:1;
		 uint8_t BIT7:1;
		 uint8_t BIT8:1;
		 uint8_t BIT9:1;
		 uint8_t BIT10:1;
		 uint8_t BIT11:1;
		 uint8_t BIT12:1;
		 uint8_t BIT13:1;
		 uint8_t BIT14:1;
		 uint8_t BIT15:1;
		 uint8_t BIT16:1;
		 uint8_t BIT17:1;
		 uint8_t BIT18:1;
		 uint8_t BIT19:1;
		 uint8_t BIT20:1;
		 uint8_t BIT21:1;
		 uint8_t BIT22:1;
		 uint8_t BIT23:1;
		 uint8_t BIT24:1;
		 uint8_t BIT25:1;
		 uint8_t BIT26:1;
		 uint8_t BIT27:1;
		 uint8_t BIT28:1;
		 uint8_t BIT29:1;
		 uint8_t BIT30:1;
		 uint8_t BIT31:1;
		 uint8_t BIT32:1;

	 }bits;
}Flag;

// Flag variable
extern Flag flag0;

// Temp variable
extern int globalTemp;

// SCR reg
extern uint16_t SCR;

//TIM1 variables
extern uint16_t TIM1_CCRValue4;
extern uint16_t TIM1_changeDelay;
extern int TIM1CaptureValue1;
extern int TIM1CaptureValue2;
extern int TIM1CaptureValue3;
extern int TIM1CaptureValue4;

// TIM4 variables
extern uint32_t TIM4_IC1_PreviousValue;
extern uint32_t TIM4_IC1_LowWidth;
extern uint32_t TIM4_IC1_HighWidth;
extern uint32_t TIM4_IC2_PreviousValue;
extern uint32_t TIM4_IC2_LowWidth;
extern uint32_t TIM4_IC2_HighWidth;
extern uint32_t TIM4_IC3_PreviousValue;
extern uint32_t TIM4_IC3_LowWidth;
extern uint32_t TIM4_IC3_HighWidth;
extern uint32_t TIM4_IC4_PreviousValue;
extern uint32_t TIM4_IC4_LowWidth;
extern uint32_t TIM4_IC4_HighWidth;

// TIM8 variables
extern uint32_t TIM8_IC1_PreviousValue;
extern uint32_t TIM8_IC1_LowWidth;
extern uint32_t TIM8_IC1_HighWidth;
extern uint32_t TIM8_IC2_PreviousValue;
extern uint32_t TIM8_IC2_LowWidth;
extern uint32_t TIM8_IC2_HighWidth;
extern uint32_t TIM8_IC3_PreviousValue;
extern uint32_t TIM8_IC3_LowWidth;
extern uint32_t TIM8_IC3_HighWidth;
extern uint32_t TIM8_IC4_PreviousValue;
extern uint32_t TIM8_IC4_LowWidth;
extern uint32_t TIM8_IC4_HighWidth;

//GPS variables
extern uint16_t GPS_Latitude;
extern uint16_t GPS_Longitude;
extern uint16_t GPS_Altitude;
extern uint16_t GPS_Speed;
extern uint16_t GPS_TrackAngle;
extern uint16_t GPS_SattelitesStatus;



// ADC trigger timer
extern uint32_t ADC_TriggerTimer;

//DMA variables
extern volatile uint8_t UART2DMAbuffer[20];

// DAC variables
extern uint32_t DAC1_TIM6reloadValue;
extern const uint16_t Sine12bit[32];


#endif /* VAR_H_ */
