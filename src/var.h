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
	 }bits;
}Flag;

// Float/char/uint32 typedef
typedef union
{
	float64_t f64;
	float32_t f32[2];
	unsigned char ch[8];
	uint32_t ui32[2];
	int32_t i32[2];
	uint16_t ui16[4];
	int16_t i16[4];
	struct
	{
	    unsigned int mantisa : 23;
	    unsigned int exponent : 8;
	    unsigned int sign : 1;
	} parts;
}__attribute__((aligned(4),packed)) CONVERTNUM;


extern volatile uint16_t mainLoopState;

extern FUSION_CORE fusionData;

// Flight data
extern FLIGHT_CORE FCFlightData;
// R/C variable
extern RCDATA RCData;
// Flight check interval
extern uint32_t ui32FlightCheckCounter;
extern uint32_t ui32FlightCheckInterval;


// Flag variable
extern volatile Flag flag0;
extern volatile Flag flag1;
extern volatile Flag APStatus1;
extern volatile Flag APStatus2;

// A/D variables
extern uint16_t AIn0;
extern uint16_t AIn1;
extern uint16_t AIn2;
extern uint16_t AIn3;


extern volatile char* fileBuffer;
extern volatile float32_t globalFloatTemp;
// Temp variable
extern volatile int globalTemp;

// Global set variable, can be set over comm
extern volatile float32_t globalVar;

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

// TIM9 variables
extern volatile uint32_t TIM9_IC1_PreviousValue;
extern volatile uint32_t TIM9_IC1_LowWidth;
extern volatile uint32_t TIM9_IC1_HighWidth;

//GPS variables
extern volatile uint16_t GPS_Latitude;
extern volatile uint16_t GPS_Longitude;
extern volatile uint16_t GPS_Altitude;
extern volatile uint16_t GPS_Speed;
extern volatile uint16_t GPS_TrackAngle;
extern volatile uint16_t GPS_SattelitesStatus;

// ADC trigger timer
extern volatile uint32_t ADC_TriggerTimer;

// DAC variables
extern volatile uint32_t DAC1_TIM6reloadValue;
extern const uint16_t Sine12bit[32];
// SD card variables
extern volatile uint8_t SD_TimerCount;
// SD card buffers - 2 x 2 kb
extern char SD_Buffer1[2048];
extern char SD_Buffer2[2048];
extern uint32_t SD_Buf1Count;
extern uint32_t SD_Buf2Count;

// FatFS variables
extern FATFS FileSystemObject;
extern DSTATUS driveStatus;
extern FIL logFile;
extern uint8_t FatFS_FlushBuffer;

// LED count variable
extern volatile uint16_t LED_ToggleCount;

// Buffer for data from PC to uC for USB
extern uint8_t Buffer[64];
extern char StringBuffer[161];			// Buffer for string manipulation
extern char FSBuffer[FATFS_BUFF_SIZE];	// Pointer to buffer for file write

// UART2 buffer
extern uint8_t UART2Buffer[1024];

extern volatile CONVERTNUM convertNumFormat;

extern volatile uint32_t fastDataSelect;

extern volatile uint16_t errorUpdateInterval;

// Signal strength variables
extern volatile uint32_t signalStrengthCount;

#endif /* VAR_H_ */
