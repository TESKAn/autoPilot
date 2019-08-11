/*
 * var.c
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#include "allinclude.h"


// Variable for main loop state execution machine

 uint16_t mainLoopState = 0;
 uint16_t ui16MainLoopVar = 0;

// Fusion data
FUSION_CORE fusionData;

// Flight data
FLIGHT_CORE FCFlightData;
// R/C variable
RCDATA RCData;
// SPI variables
FUSION_SPIDATA *SPI_SensorBuf;
FUSION_SPIDATA SPI_SensorBufAcc;
FUSION_SPIDATA SPI_SensorBufGyro;
FUSION_SPIDATA SPI_SensorBufMag;
FUSION_SPIDATA SPI_SensorBufBaro;

// Flag variables
volatile Flag flag0;
volatile Flag flag1;


// A/D variables
uint16_t AIn0 = 0;
uint16_t AIn1 = 0;
uint16_t AIn2 = 0;
uint16_t AIn3 = 0;

//TIM1 variables
volatile int TIM1CaptureValue1 = 0;
volatile int TIM1CaptureValue2 = 0;
volatile int TIM1CaptureValue3 = 0;
volatile int TIM1CaptureValue4 = 0;

// TIM3 variables
volatile uint32_t TIM3_IC1_PreviousValue = 0;
volatile uint32_t TIM3_IC1_LowWidth = 0;
volatile uint32_t TIM3_IC1_HighWidth = 0;
volatile uint32_t TIM3_IC2_PreviousValue = 0;
volatile uint32_t TIM3_IC2_LowWidth = 0;
volatile uint32_t TIM3_IC2_HighWidth = 0;

// TIM4 variables
volatile uint32_t TIM4_IC1_PreviousValue = 0;
volatile uint32_t TIM4_IC1_LowWidth = 0;
volatile uint32_t TIM4_IC1_HighWidth = 0;
volatile uint32_t TIM4_IC2_PreviousValue = 0;
volatile uint32_t TIM4_IC2_LowWidth = 0;
volatile uint32_t TIM4_IC2_HighWidth = 0;
volatile uint32_t TIM4_IC3_PreviousValue = 0;
volatile uint32_t TIM4_IC3_LowWidth = 0;
volatile uint32_t TIM4_IC3_HighWidth = 0;
volatile uint32_t TIM4_IC4_PreviousValue = 0;
volatile uint32_t TIM4_IC4_LowWidth = 0;
volatile uint32_t TIM4_IC4_HighWidth = 0;

// TIM8 variables
volatile uint32_t TIM8_IC1_PreviousValue = 0;
volatile uint32_t TIM8_IC1_LowWidth = 0;
volatile uint32_t TIM8_IC1_HighWidth = 0;
volatile uint32_t TIM8_IC2_PreviousValue = 0;
volatile uint32_t TIM8_IC2_LowWidth = 0;
volatile uint32_t TIM8_IC2_HighWidth = 0;
volatile uint32_t TIM8_IC3_PreviousValue = 0;
volatile uint32_t TIM8_IC3_LowWidth = 0;
volatile uint32_t TIM8_IC3_HighWidth = 0;
volatile uint32_t TIM8_IC4_PreviousValue = 0;
volatile uint32_t TIM8_IC4_LowWidth = 0;
volatile uint32_t TIM8_IC4_HighWidth = 0;

// TIM9 variables
volatile uint32_t TIM9_IC1_PreviousValue = 0;
volatile uint32_t TIM9_IC1_LowWidth = 0;
volatile uint32_t TIM9_IC1_HighWidth = 0;

// SD card variables
volatile uint8_t SD_TimerCount = 0;
// SD card buffers - 2 x 2 kb
char SD_Buffer1[2048];
char SD_Buffer2[2048];
uint32_t SD_Buf1Count = 0;
uint32_t SD_Buf2Count = 0;

// FatFS variables
FATFS FileSystemObject;
DSTATUS driveStatus;
FIL logFile;
uint8_t FatFS_FlushBuffer = 0;

// LED count variable
volatile uint16_t LED_ToggleCount = 0;

char FSBuffer[FATFS_BUFF_SIZE];	// Pointer to buffer for file write



// Signal strength variables
volatile uint32_t signalStrengthCount = 0;

uint32_t ui32CANTime = 0;
uint32_t ui32SendAHRSDataTime = 0;

uint8_t mavlinkBuffer[384];
mavlink_message_t mavlinkMessageDataRX;
mavlink_message_t mavlinkMessageDataTX;
mavlink_status_t mavlinkStatusData;
uint8_t mavlinkSendBusy;

uint16_t ui16SendMavlink10Hz = 0;
uint16_t ui16SendMavlink5Hz = 0;
uint16_t ui16SendMavlink2Hz = 0;
uint16_t ui16SendMavlink1Hz = 0;

uint16_t ui16MavlinkBatteryVoltages[10];

uint16_t ui16MavlinkQueueState = 0;
RING_BUFFER32 rb32MavlinkTXQueue;
uint32_t ui32MavlinkBuf[32];

T32BITVARS t32CANVar;

int32_t i32CurrentMavlinkMessage = 0;
int32_t i32CurrentMavlinkCommand = 0;

RING_BUFFER32 rb32SensorTXQueue;
uint32_t ui32SensorBuf[32];

int16_t i16SPITestData = 0;
uint16_t ui16SPITestAddress = 0;
uint16_t ui16SPITestDevice = 1;

uint8_t ui8MagDataTransfer[13];
uint8_t ui8MagDataTransferTime = 0;
uint8_t ui8ui8MagDataTransferCount = 0;
