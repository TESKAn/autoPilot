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

// check main loop state
int16_t CheckMainLoopStates()
{
	// Main loop switch
	switch(mainLoopState)
	{
		case 0:
		{
			break;
		}
		case 1:
		{
			LED_ERR_ON;
			LED_OK_ON;
			LED_RUN_ON;

			break;
		}
		case 2:
		{
			LED_ERR_OFF;
			LED_OK_OFF;
			LED_RUN_OFF;
			break;
		}
		case 3:
		{
			mainLoopState = 0;
			MPU_COMM_ENABLED = 0;
			while(I2C2_WAITINGDATA ) {}
			MPU6000_GyroSelfTest(DISABLE);
			MPU_COMM_ENABLED = 1;

			break;
		}
		case 4:
		{
			mainLoopState = 0;
			MPU_COMM_ENABLED = 0;
			while(I2C2_WAITINGDATA ) {}
			MPU6000_AccSelfTest(ENABLE);
			MPU_COMM_ENABLED = 1;

			break;
		}
		case 5:
		{
			mainLoopState = 0;
			MPU_COMM_ENABLED = 0;
			while(I2C2_WAITINGDATA ) {}
			MPU6000_AccSelfTest(DISABLE);
			MPU_COMM_ENABLED = 1;

			break;
		}
		case 6:
		{
			mainLoopState = 0;
			MPU_COMM_ENABLED = 0;
			while(I2C2_WAITINGDATA ) {}
			MPU6000_ReadFTValues();
			MPU_COMM_ENABLED = 1;

			break;
		}
		case 7:
		{
			mainLoopState = 0;
			// Acc offsets
			acc_updateOffsets(&fusionData._accelerometer);
			break;
		}
		case 8:
		{
			// All gains
			acc_updateGains(&fusionData._accelerometer, 0);
			mainLoopState = 0;
			break;
		}
		case 9:
		{
			// X gain
			acc_updateGains(&fusionData._accelerometer, 1);
			mainLoopState = 0;
			break;
		}
		case 10:
		{
			// Y gain
			acc_updateGains(&fusionData._accelerometer, 2);
			mainLoopState = 0;
			break;
		}
		case 11:
		{
			// Z gain
			acc_updateGains(&fusionData._accelerometer, 3);
			mainLoopState = 0;
			break;
		}
		case 12:
		{
			// Send reset signal
			CAN_SendRESET(21);
			mainLoopState = 0;
			break;
		}
		case 13:
		{
			// Write bat low value
			t32CANVar.ui16[0] = ui16MainLoopVar;
			CAN_SendRegValue(CAN_WRITE_REG_BATLOW, &t32CANVar, 20);
			CAN_SendRegValue(CAN_WRITE_REG_BATLOW, &t32CANVar, 21);
			CAN_SendRegValue(CAN_WRITE_REG_BATLOW, &t32CANVar, 22);
			CAN_SendRegValue(CAN_WRITE_REG_BATLOW, &t32CANVar, 23);
			mainLoopState = 0;
			break;
		}
		case 14:
		{
			// Enable motor
			enableMotors(1);
			mainLoopState = 0;
			break;
		}
		case 15:
		{
			// Disable motor
			enableMotors(0);
			mainLoopState = 0;
			break;
		}
		case 16:
		{
			// Store parameters
			storeMotorParams();
			mainLoopState = 0;
			break;
		}
		case 17:
		{
			// Write min RPM
			t32CANVar.ui16[0] = ui16MainLoopVar;
			CAN_SendRegValue(CAN_WRITE_REG_MINRPM, &t32CANVar, 20);
			CAN_SendRegValue(CAN_WRITE_REG_MINRPM, &t32CANVar, 21);
			CAN_SendRegValue(CAN_WRITE_REG_MINRPM, &t32CANVar, 22);
			CAN_SendRegValue(CAN_WRITE_REG_MINRPM, &t32CANVar, 23);
			mainLoopState = 0;
			break;
		}
		case 18:
		{
			// Store mag buffer address
			SPI_SensorBuf = &SPI_SensorBufMag;
			SPI_SensorBuf->ui8Device = MAG_DEV_CS;
			// Read data from mag
			Sensor_SPIReadDMA();
			mainLoopState = 0;
			break;
		}
		case 19:
		{
			// Store baro buffer address
			SPI_SensorBufMag.ui8TxBuf[0] = 0x80 | SPI_SensorBufMag.ui16StartReg;
			SPI_SensorBufMag.ui8TxBuf[1] = SPI_SensorBufMag.ui8TxBuf[0] + 1;
			SPI_SensorBufMag.ui8TxBuf[2] = SPI_SensorBufMag.ui8TxBuf[1] + 1;
			SPI_SensorBufMag.ui8TxBuf[3] = SPI_SensorBufMag.ui8TxBuf[2] + 1;
			SPI_SensorBufMag.ui8TxBuf[4] = SPI_SensorBufMag.ui8TxBuf[3] + 1;
			SPI_SensorBufMag.ui8TxBuf[5] = SPI_SensorBufMag.ui8TxBuf[4] + 1;
			SPI_SensorBufMag.ui8TxBuf[6] = SPI_SensorBufMag.ui8TxBuf[5] + 1;
			SPI_SensorBufMag.ui8TxBuf[7] = SPI_SensorBufMag.ui8TxBuf[6] + 1;
			SPI_SensorBufMag.ui8TxBuf[8] = SPI_SensorBufMag.ui8TxBuf[7] + 1;
			SPI_SensorBufMag.ui8TxBuf[9] = SPI_SensorBufMag.ui8TxBuf[8] + 1;
			SPI_SensorBufMag.ui8TxBuf[10] = SPI_SensorBufMag.ui8TxBuf[9] + 1;
			SPI_SensorBufMag.ui8TxBuf[11] = SPI_SensorBufMag.ui8TxBuf[10] + 1;
			SPI_SensorBufMag.ui8TxBuf[12] = SPI_SensorBufMag.ui8TxBuf[11] + 1;
			SPI_SensorBufMag.ui8TxBuf[13] = SPI_SensorBufMag.ui8TxBuf[12] + 1;
			SPI_SensorBufMag.ui8TxBuf[14] = SPI_SensorBufMag.ui8TxBuf[13] + 1;
			SPI_SensorBufMag.ui8TxBuf[15] = SPI_SensorBufMag.ui8TxBuf[14] + 1;

			mainLoopState = 0;
			break;
		}
		case 20:
		{
			RB32_push(&rb32SensorTXQueue, MAG_GET_DATA);
			mainLoopState = 0;
			break;
		}
		case 21:
		{
			RB32_push(&rb32SensorTXQueue, BARO_GET_DATA);
			mainLoopState = 0;
			break;
		}
		case 22:
		{
			if(SPI_INC_ADDRESS_WRITE) ui16SPITestAddress++;
			RB32_push(&rb32SensorTXQueue, SENSOR_WRITE_REG);
			//RB32_push(&rb32SensorTXQueue, SENSOR_WRITE_REG);
			//i16SPITestData = Sensor_SPIWrite(ui16SPITestDevice, ui16SPITestAddress, i16SPITestData);
			mainLoopState = 0;
			break;
		}
		case 23:
		{
			if(SPI_INC_ADDRESS_READ) ui16SPITestAddress++;
			RB32_push(&rb32SensorTXQueue, SENSOR_READ_REG);
			//RB32_push(&rb32SensorTXQueue, SENSOR_READ_REG);
			//i16SPITestData = Sensor_SPIRead(ui16SPITestDevice, ui16SPITestAddress);
			mainLoopState = 0;
			break;
		}
		case 24:
		{
			RB32_push(&rb32SensorTXQueue, SENSOR_READ_WORD);
			mainLoopState = 0;
			break;
		}
		case 25:
		{
			CAN_SendENABLE(0, CAN_MOTOR_ALL_ID);
			mainLoopState = 0;
			break;
		}
		case 26:
		{
			if(0 == mavlinkSendBusy)
			{
				/*
				//ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 1, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 25, &FCFlightData.batMon.ui6MavLinkVoltage, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1, -1, 	MAV_BATTERY_CHARGE_STATE_OK);
				ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
				transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);*/
				mavlinkSendBusy = 1;
			}
			mainLoopState = 0;
			break;
		}
		case 27:
		{
			matrix3_init(1, &fusionData._fusion_DCM);
			mainLoopState = 0;
			break;
		}
		default:
		{
			mainLoopState = 0;
			break;
		}
	}
	return 0;
}

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
}

// Returns system time, resolution is 10 usec
uint32_t getSystemTime(void)
{
	// Return timer2 value - counting from 0 on reset to 0xffffffff in 1 us increments, 1.2 hours total
	return READ_SYS_TIME;
}


// Return system time in milliseconds
float32_t getFTime(void)
{
	uint32_t ui32SysTime;
	float32_t time = 0;
	ui32SysTime = READ_SYS_TIME;
	time = (float32_t)ui32SysTime / 1000000;
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
	uint32_t deltaTime = 0;
	uint32_t time = getSystemTime();
	uint32_t endTime = 0;

	endTime = time + (ms * 1000);
	deltaTime = time;
	while(endTime > time)
	{
		time = getSystemTime();
		if(time == deltaTime)
		{
			// Error - time is not counting, break the loop
			Delaynus(ms * 1000);
			break;
		}
		deltaTime = time;
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
