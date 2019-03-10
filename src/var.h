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
	uint8_t ch[8];
	int8_t i8[8];
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

typedef union
{
	union
	{
		struct
		{
			int8_t ui8[2];
		}bytes;
		struct
		{
			int8_t i8[2];
		}ints;
		//UWord16 uw16;
		uint16_t ui16;
		int16_t i16;
		//Frac16 f16;
		//Word16 w16;
	};
}__attribute__((aligned(4),packed))  T16BITVARS;

typedef union
{
	union
	{
		struct
		{
			uint8_t ui8[4];
		};
		struct
		{
			int8_t i8[4];
		};
		struct
		{
			uint16_t ui16[2];
		};
		struct
		{
			int16_t i16[2];
		};
		uint32_t ui32;
		int32_t i32;
		float32_t f;
	};
}__attribute__((aligned(4),packed))  T32BITVARS;

typedef union
{
    uint32_t u;
    float f;
}__attribute__((aligned(4),packed))  FP32;

// Structure that holds all relevant data for motor
typedef struct tagCOMMMOTOR
{
	uint8_t errStatus;
	uint8_t ui8FreshData;
	union
	{
		uint8_t ui8REGSData[64];				// Main data structure
		struct
		{
			// Some params
			// Errors
			uint16_t ui16Errors;			// 0
			uint16_t ui16ModelNumber;		// 2
			uint8_t ui8FirmwareVersion;	// 4
			uint8_t ui8ID;				// 5
			uint8_t ui8BaudRate;			// 6
			uint8_t ui8Empty;				// 7


			// Motor state - idle, run, error
			uint16_t ui16State;			// 8

			// Status of the motor
			int16_t i16UIn;				// 10
			int16_t i16IIn;				// 12
			int16_t i16PIn;				// 14
			int16_t i16RPM;				// 16
			int16_t i16Position;			// 18
			uint8_t ui8PresentTemperature;	//19
			// Future expansion
			uint8_t ui8Empty1[11];			// 32 bytes total

			// Motor control
			// Arm
			uint8_t ui8Armed;				// 32
			// Park
			uint8_t ui8Park;				// 33
			// Reverse rotation
			uint8_t ui8ReverseRotation;	// 34
			uint8_t uiEmpty2;
			// Park position
			int16_t i16ParkPosition;		// 36

			int16_t i16SetRPM;			// 38
			int16_t i16MaxRPM;			// 40
			int16_t i16MinRPM;			// 42

			// PWM input
			uint8_t ui8MeasurePWMMin;		// 44
			uint8_t ui8MeasurePWMMax;		// 45
			uint8_t ui8UsePWMIN;			// 46
			uint8_t uiEmpty3;				// 47
			int16_t i16PWMMin;			// 48
			int16_t i16PWMMax;			// 50
			int16_t i16ZeroSpeedPWM;		// 52
			int16_t i16CurrentPWM;		// 54	//55 bytes total

			uint8_t uiEmpty4[8];					// 64 bytes total

		}REGS;
	};

}__attribute__((aligned(4),packed)) COMMMOTOR;

// Timing variables
extern uint32_t ui32StartTime;
extern uint32_t ui32EndTime;
extern uint32_t ui32ElapsedTime;
extern uint32_t ui32LastSensorUpdateTime;
extern uint32_t ui32SensorUpdateInterval;


extern uint16_t mainLoopState;
extern uint16_t ui16MainLoopVar;
extern uint16_t servoMovePosition;
extern float32_t motorFRSpeed;
extern uint16_t readRS485Data;
extern uint32_t ui32MainLoopCanVar;
extern uint32_t ui32MainLoopCanVar1;

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
//extern uint8_t UART2Buffer[1024];
extern uint8_t ui8BufferToSend;

extern volatile CONVERTNUM convertNumFormat;

extern volatile uint32_t fastDataSelect;

extern volatile uint16_t errorUpdateInterval;

// Signal strength variables
extern volatile uint32_t signalStrengthCount;

extern uint32_t ui32TestVar;

extern uint32_t ui32CANTime;
extern uint32_t ui32SendAHRSDataTime;

extern uint8_t mavlinkBuffer[384];
extern mavlink_message_t mavlinkMessageDataRX;
extern mavlink_message_t mavlinkMessageDataTX;
extern mavlink_status_t mavlinkStatusData;
extern uint8_t mavlinkSendBusy;

extern uint16_t ui16SendMavlink10Hz;
extern uint16_t ui16SendMavlink5Hz;
extern uint16_t ui16SendMavlink2Hz;
extern uint16_t ui16SendMavlink1Hz;

extern uint16_t ui16MavlinkBatteryVoltages[10];

extern uint16_t ui16MavlinkQueueState;
extern RING_BUFFER32 rb32MavlinkTXQueue;
extern uint32_t ui32MavlinkBuf[32];

extern T32BITVARS t32CANVar;

extern int32_t i32CurrentMavlinkMessage;
extern int32_t i32CurrentMavlinkCommand;

#endif /* VAR_H_ */
