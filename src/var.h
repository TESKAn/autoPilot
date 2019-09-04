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

#include "sensors/sensor_typedefs.h"
#include "sensors/sensors_fusion.h"


extern uint16_t mainLoopState;
extern uint16_t ui16MainLoopVar;

extern FUSION_CORE fusionData;

// Flight data
extern FLIGHT_CORE FCFlightData;
// R/C variable
extern RCDATA RCData;

// SPI variables
extern FUSION_SPIDATA *SPI_SensorBuf;
extern FUSION_SPIDATA SPI_SensorBufAcc;
extern FUSION_SPIDATA SPI_SensorBufGyro;
extern FUSION_SPIDATA SPI_SensorBufMag;
extern FUSION_SPIDATA SPI_SensorBufBaro;
// Flag variable
extern volatile Flag flag0;
extern volatile Flag flag1;

// A/D variables
extern uint16_t AIn0;
extern uint16_t AIn1;
extern uint16_t AIn2;
extern uint16_t AIn3;

//TIM1 variables
extern volatile int TIM1CaptureValue1;
extern volatile int TIM1CaptureValue2;
extern volatile int TIM1CaptureValue3;
extern volatile int TIM1CaptureValue4;

// TIM3 variables
extern volatile uint32_t TIM3_IC1_PreviousValue;
extern volatile uint32_t TIM3_IC1_LowWidth;
extern volatile uint32_t TIM3_IC1_HighWidth;
extern volatile uint32_t TIM3_IC2_PreviousValue;
extern volatile uint32_t TIM3_IC2_LowWidth;
extern volatile uint32_t TIM3_IC2_HighWidth;

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

extern char FSBuffer[FATFS_BUFF_SIZE];	// Pointer to buffer for file write

// Signal strength variables
extern volatile uint32_t signalStrengthCount;

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

extern RING_BUFFER32 rb32SensorTXQueue;
extern uint32_t ui32SensorBuf[32];

extern int16_t i16SPITestData;
extern uint16_t ui16SPITestAddress;
extern uint16_t ui16SPITestDevice;

extern uint8_t ui8MagDataTransfer[13];
extern uint8_t ui8MagDataTransferTime;
extern uint8_t ui8ui8MagDataTransferCount;

extern MAFilter MARotationFilter;

#endif /* VAR_H_ */
