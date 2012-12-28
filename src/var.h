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
		volatile uint32_t flag;
	}flag;

	 struct
	 {
		volatile uint8_t BIT0:1;
		volatile uint8_t BIT1:1;
		volatile uint8_t BIT2:1;
		volatile uint8_t BIT3:1;
		volatile uint8_t BIT4:1;
		volatile uint8_t BIT5:1;
		volatile uint8_t BIT6:1;
		volatile uint8_t BIT7:1;
		volatile uint8_t BIT8:1;
		volatile uint8_t BIT9:1;
		volatile uint8_t BIT10:1;
		volatile uint8_t BIT11:1;
		volatile uint8_t BIT12:1;
		volatile uint8_t BIT13:1;
		volatile uint8_t BIT14:1;
		volatile uint8_t BIT15:1;
		volatile uint8_t BIT16:1;
		volatile uint8_t BIT17:1;
		volatile uint8_t BIT18:1;
		volatile uint8_t BIT19:1;
		volatile uint8_t BIT20:1;
		volatile uint8_t BIT21:1;
		volatile uint8_t BIT22:1;
		volatile uint8_t BIT23:1;
		volatile uint8_t BIT24:1;
		volatile uint8_t BIT25:1;
		volatile uint8_t BIT26:1;
		volatile uint8_t BIT27:1;
		volatile uint8_t BIT28:1;
		volatile uint8_t BIT29:1;
		volatile uint8_t BIT30:1;
		volatile uint8_t BIT31:1;
		volatile uint8_t BIT32:1;

	 }bits;
}Flag;

// Flag variable
extern volatile Flag flag0;

// Temp variable
extern volatile int globalTemp;

// System time variable
extern volatile uint32_t systemTime;

// SCR reg
extern volatile uint16_t SCR;

//TIM1 variables
extern volatile uint16_t TIM1_CCRValue4;
extern volatile uint16_t TIM1_changeDelay;
extern volatile int TIM1CaptureValue1;
extern volatile int TIM1CaptureValue2;
extern volatile int TIM1CaptureValue3;
extern volatile int TIM1CaptureValue4;

// TIM4 variables
extern volatile uint32_t TIM4_IC1_PreviousValue;
extern volatile uint32_t TIM4_IC1_LowWidth;
extern volatile uint32_t TIM4_IC1_HighWidth;
extern volatile uint32_t TIM4_IC2_PreviousValue;
extern volatile uint32_t TIM4_IC2_LowWidth;
extern volatile uint32_t TIM4_IC2_HighWidth;
extern volatile uint32_t TIM4_IC3_PreviousValue;
extern volatile uint32_t TIM4_IC3_LowWidth;
extern volatile uint32_t TIM4_IC3_HighWidth;
extern volatile uint32_t TIM4_IC4_PreviousValue;
extern volatile uint32_t TIM4_IC4_LowWidth;
extern volatile uint32_t TIM4_IC4_HighWidth;

// TIM8 variables
extern volatile uint32_t TIM8_IC1_PreviousValue;
extern volatile uint32_t TIM8_IC1_LowWidth;
extern volatile uint32_t TIM8_IC1_HighWidth;
extern volatile uint32_t TIM8_IC2_PreviousValue;
extern volatile uint32_t TIM8_IC2_LowWidth;
extern volatile uint32_t TIM8_IC2_HighWidth;
extern volatile uint32_t TIM8_IC3_PreviousValue;
extern volatile uint32_t TIM8_IC3_LowWidth;
extern volatile uint32_t TIM8_IC3_HighWidth;
extern volatile uint32_t TIM8_IC4_PreviousValue;
extern volatile uint32_t TIM8_IC4_LowWidth;
extern volatile uint32_t TIM8_IC4_HighWidth;

//GPS variables
extern volatile uint16_t GPS_Latitude;
extern volatile uint16_t GPS_Longitude;
extern volatile uint16_t GPS_Altitude;
extern volatile uint16_t GPS_Speed;
extern volatile uint16_t GPS_TrackAngle;
extern volatile uint16_t GPS_SattelitesStatus;



// ADC trigger timer
extern volatile uint32_t ADC_TriggerTimer;

//DMA variables
extern volatile uint8_t UART2DMAbuffer[20];

// DAC variables
extern volatile uint32_t DAC1_TIM6reloadValue;
extern const uint16_t Sine12bit[32];
// SD card variables
extern volatile uint8_t SD_TimerCount;

// FatFS variables
extern FATFS FileSystemObject;
extern DSTATUS driveStatus;
extern FIL logFile;
extern uint8_t FatFS_FlushBuffer;

// LED count variable
extern volatile uint16_t LED_ToggleCount;

// Buffer for data from PC to uC for USB
extern uint8_t Buffer[64];
extern char FSBuffer[FATFS_BUFF_SIZE];	// Pointer to buffer for file write

#endif /* VAR_H_ */
