/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"
#include <string.h>
#include <stdlib.h>
#include "sensors/altimeter.h"

// Check RC inputs timing
int16_t CheckRCInputTimeouts()
{
	int i = 0;
	int goodChannels = 0;
	for(i=0; i<8;i++)
	{
		RCData.ch[i].PWM_Timeout++;
		if(100 < RCData.ch[i].PWM_Timeout)
		{
			RCData.ch[i].PWM_Timeout = 100;
			RCData.ch[i].PWM_Good = 0;
		}
		else
		{
			RCData.ch[i].PWM_Good = 1;
			goodChannels++;
		}
	}

	if(6 <= goodChannels)
	{
		RCData.inputs_ok = 1;
	}
	else
	{
		RCData.inputs_ok = 0;
	}

	return 0;
}


// Calibrate sensors
void calibrateI2CSensors(void)
{

}

int16_t checkMotorHealth()
{
	FLIGHT_MOTOR* FMotorData;
	int i = 0;

	for(i=0; i<4; i++)
	{
		switch(i)
		{
			case 0:
			{
				FMotorData = &FCFlightData.MOTORS.FR;
				break;
			}
			case 1:
			{
				FMotorData = &FCFlightData.MOTORS.FL;
				break;
			}
			case 2:
			{
				FMotorData = &FCFlightData.MOTORS.RR;
				break;
			}
			case 3:
			{
				FMotorData = &FCFlightData.MOTORS.RL;
				break;
			}
		}
		// Check enable/disable
		if(1 == FMotorData->ui8Enable)
		{
			// Is motor disabled?
			if(0 == FMotorData->ui8Enabled)
			{
				if(0 == FMotorData->i16EnableDisableTimeout)
				{
					// Mark enable sent
					//FMotorData->ui8Enable = 2;
					// Send enable
					CAN_SendENABLE(1, FMotorData->ui8MotorID);
					FMotorData->i16EnableDisableTimeout = 50;
				}
				else
				{
					FMotorData->i16EnableDisableTimeout--;
				}
			}
		}
		else if(0 == FMotorData->ui8Enable)
		{
			// Is motor enabled?
			if(1 == FMotorData->ui8Enabled)
			{
				if(0 == FMotorData->i16EnableDisableTimeout)
				{
					// Mark enable sent
					//FMotorData->ui8Enable = 3;
					// Send enable
					CAN_SendENABLE(0, FMotorData->ui8MotorID);
					FMotorData->i16EnableDisableTimeout = 50;
				}
				else
				{
					FMotorData->i16EnableDisableTimeout--;
				}
			}
		}
		// Check motor health
		// Motor health is not OK
		switch(FMotorData->ui8Health)
		{
			case 0:
			{
				FMotorData->ui8ResetSent = 0;
				break;
			}
			case 1:
			{
				break;
			}
			case 2:
			{
				if(0 == FMotorData->ui8ResetSent)
				{
					FMotorData->ui8ResetSent = 1;
					CAN_SendRESET(FMotorData->ui8MotorID);
				}
				break;
			}
			case 3:
			{
				if(0 == FMotorData->ui8ResetSent)
				{
					FMotorData->ui8ResetSent = 1;
					CAN_SendRESET(FMotorData->ui8MotorID);
				}
				break;
			}
		}
	}
	return 0;
}

void refreshMotorRPM()
{
	// Buffer to CAN
	CAN_SendRPM_single(FCFlightData.MOTORS.FR.i16ReqRPM, FCFlightData.MOTORS.FR.ui8MotorID);
	CAN_SendRPM_single(FCFlightData.MOTORS.FL.i16ReqRPM, FCFlightData.MOTORS.FL.ui8MotorID);
	CAN_SendRPM_single(FCFlightData.MOTORS.RR.i16ReqRPM, FCFlightData.MOTORS.RR.ui8MotorID);
	CAN_SendRPM_single(FCFlightData.MOTORS.RL.i16ReqRPM, FCFlightData.MOTORS.RL.ui8MotorID);
}

void enableMotors(uint8_t ui8Enable)
{
	CAN_SendENABLE(ui8Enable, FCFlightData.MOTORS.FR.ui8MotorID);
	CAN_SendENABLE(ui8Enable, FCFlightData.MOTORS.FL.ui8MotorID);
	CAN_SendENABLE(ui8Enable, FCFlightData.MOTORS.RR.ui8MotorID);
	CAN_SendENABLE(ui8Enable, FCFlightData.MOTORS.RL.ui8MotorID);
}

int16_t storeMotorParams()
{
	t32CANVar.ui32 = 1;
	CAN_SendRegValue(CAN_WRITE_TOFLASH, &t32CANVar, 20);
	CAN_SendRegValue(CAN_WRITE_TOFLASH, &t32CANVar, 21);
	CAN_SendRegValue(CAN_WRITE_TOFLASH, &t32CANVar, 22);
	CAN_SendRegValue(CAN_WRITE_TOFLASH, &t32CANVar, 23);
	return 0;
}

// Update PWM out values
void refreshPWMOutputs(void)
{
	TIM_SetCompare4(TIM1, RCData.ch[0].PWMOUT);
	TIM_SetCompare3(TIM1, RCData.ch[1].PWMOUT);
	TIM_SetCompare2(TIM1, RCData.ch[2].PWMOUT);
	TIM_SetCompare1(TIM1, RCData.ch[3].PWMOUT);
	TIM_SetCompare4(TIM3, RCData.ch[4].PWMOUT);
	TIM_SetCompare3(TIM3, RCData.ch[5].PWMOUT);
	TIM_SetCompare2(TIM3, RCData.ch[6].PWMOUT);
	TIM_SetCompare1(TIM3, RCData.ch[7].PWMOUT);
	TIM_SetCompare4(TIM2, RCData.ch[8].PWMOUT);
	TIM_SetCompare3(TIM2, RCData.ch[9].PWMOUT);
	TIM_SetCompare2(TIM2, RCData.ch[10].PWMOUT);
	TIM_SetCompare1(TIM2, RCData.ch[11].PWMOUT);
}

// Returns system time, resolution is 10 usec
uint32_t getSystemTime(void)
{
	uint32_t time = 0;
	uint32_t timeFrac = 0;
	time = systemTime * 100;	// 1 time tick is 10 usec
	timeFrac = (uint32_t)(TIM14->CNT);
	timeFrac = timeFrac / 10;	// Period is 1000 usec
	time = time + timeFrac;
	return time;
}


// Return system time in milliseconds
float32_t getFTime(void)
{
	float32_t time = 0;
	float32_t timeFrac = 0;
	time = systemTime;
	timeFrac = (float32_t)(TIM14->CNT);
	timeFrac = timeFrac / 1000;
	time = time + timeFrac;
	return time;
}

void extPeripheralInit(void)
{
	// Turn sensor power ON
	SENSOR_POWER_ON;
	// Initialize SD card
	FS_Initialize();

	// Setup GPS
	//GPSSetDataOutput(getSystemTime());

	Delayms(100);
	// Setup sensors

	// set PS busy
	PSBUSY = 1;
	// Short delay
	Delaynus(5000);

	sensorInit();

	ADC_ENABLED = 1;
	// Mark sensors initiated
	EXTSENS_INIT_DONE = 1;
	// Mark null sensor
	//EXTSENS_NULLING_GYRO = 1;

	Delayms(100);

}

void Delayms(uint32_t ms)
{
	uint32_t time = systemTime - 1;
	uint32_t deltaTime = 0;
	while(deltaTime < ms)
	{
		Delaynus(1500);
		if(systemTime == time)
		{
			// Error - time is not counting, break the loop
			Delaynus(ms * 1000);
			break;
		}
		deltaTime = systemTime - time;
	}
}

/*
 * Function Name  : Delaynus
 * Description    : Inserts a delay time abort nus.
 * Input          :nus
 * Output         : None
 * Return         : None
 */
void Delaynus(vu32 nus)
{
    u8 nCount;

    while (nus--)
    {
        for (nCount = 6; nCount != 0; nCount--);
    }
}

void transferDMA_USART1(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART1 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART1);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_USART1_CH;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//    USART1_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART1, &DMAInitStructure);

	//Enable DMA2 stream 7 - USART1 TX
	DMA_Cmd(DMA_USART1, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART1, DMA_IT_TC, ENABLE);
}

void transferDMA_USART2(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART2 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART2);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_4;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//    USART2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART2, &DMAInitStructure);

	//Enable DMA1 stream 0 - USART2 TX
	DMA_Cmd(DMA_USART2, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART2, DMA_IT_TC, ENABLE);
}

void transferDMA_USART3(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART3 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART3);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;//    USART3_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART3, &DMAInitStructure);

	//Enable DMA1 stream 4 - USART3 TX
	DMA_Cmd(DMA_USART3, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART3, DMA_IT_TC, ENABLE);
}

float32_t intToFloat(uint16_t whole, uint16_t frac)
{
	float32_t result = 0;
	float32_t temp = 0;
	temp = (float32_t)frac;
	while(temp > 1.0f)
	{
		temp = temp / 10;
	}
	result = (float32_t)whole + temp;
	return result;
}

// Function to convert 32 bit float to 16 bit fixed point float
// Beware of byte order.
uint16_t Float32ToFloat16(float value)
{
	T16BITVARS t16Var1;

	const FP32 f32inf = { 255UL << 23U };
	const FP32 f16inf = { 31UL << 23U };
	const FP32 magic = { 15UL << 23U };
	const uint32_t sign_mask = 0x80000000UL;
	const uint32_t round_mask = ~0xFFFUL;

	FP32 in;
	uint32_t sign;

	in.f = value;
	sign = in.u & sign_mask;
	in.u ^= sign;

	if (in.u >= f32inf.u)
	{
		t16Var1.ui16 = (in.u > f32inf.u) ? (uint16_t)0x7FFFU : (uint16_t)0x7C00U;
	}
	else
	{
	    in.u &= round_mask;
	    in.f *= magic.f;
	    in.u -= round_mask;
	    if (in.u > f16inf.u)
	    {
	        in.u = f16inf.u;
	    }
	    t16Var1.ui16 = (uint16_t)(in.u >> 13U);
	}
	t16Var1.ui16 |= (uint16_t)(sign >> 16U);

	return t16Var1.ui16;
}
