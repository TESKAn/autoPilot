/*
 * var.c
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#include "allinclude.h"

// STM studio vars
volatile int exportVars[64];

// Flag variable
volatile Flag flag0;

volatile char* fileBuffer;
volatile float32_t globalFloatTemp = 0;

// Temp variable
volatile int globalTemp = 0;

// Global set variable, can be set over comm
volatile float32_t globalVar = 0;

// System time variable
volatile uint32_t systemTime = 0;

// SCR reg
// Defined in modbus.h

//TIM1 variables
volatile uint16_t TIM1_CCRValue4 = 2200;

volatile uint16_t TIM1_changeDelay = 0;

volatile int TIM1CaptureValue1 = 0;
volatile int TIM1CaptureValue2 = 0;
volatile int TIM1CaptureValue3 = 0;
volatile int TIM1CaptureValue4 = 0;

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

// GPS variables
// Defined in modbus.h

// Power sensor variables
// Defined in modbus.h

// ADC trigger timer
volatile uint32_t ADC_TriggerTimer = 0;

//DMA variables
volatile uint8_t UART2DMAbuffer[20];

// DAC variables
volatile uint32_t DAC1_TIM6reloadValue = 0xFF;
const uint16_t Sine12bit[32] = {
                      2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
                      3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
                      599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647};
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

// Buffer for data from PC to uC for USB
uint8_t Buffer[64];
char StringBuffer[161];			// Buffer for string manipulation
char FSBuffer[FATFS_BUFF_SIZE];	// Pointer to buffer for file write

volatile C_Float floatToUint32;

volatile vector3fData gravityVector;

