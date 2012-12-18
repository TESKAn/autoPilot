/*
 * sensors.c
 *
 *  Created on: Oct 31, 2012
 *      Author: Jure
 */

#include "allinclude.h"

volatile uint16_t I2C2_ProcesState = 0;
volatile Flag I2C2_Flags;
volatile uint16_t I2C2_StartReg = 0;
volatile uint16_t I2C2_ReadData = 0;
volatile uint16_t I2C2_WriteData = 0;
volatile uint8_t I2C2_DeviceAddress = 0;
uint8_t I2C2_DMABufTX[DMA_BUF_COUNT];
uint8_t I2C2_DMABufRX[DMA_BUF_COUNT];
volatile int I2C2_DMABufTXCount = 0;
volatile int I2C2_DMABufRXCount = 0;
volatile int I2C2_PollTimer = 0;

// Initialize I2C sensors
void sensorInit()
{
	I2C2_INITDONE = 0;
	// Check if MPU is in sleep mode
	// Read reg 107
	masterReceive(MPU6000_ADDRESS, 107, I2C2_DMABufRX, 5);
	Delaynus(20000);
	// Check bit 6 - device sleep
	if((I2C2_DMABufRX[0] & _BIT7) == 0)
	{
		// Device was configured, reset MPU
		I2C2_DMABufTX[0] = 107;
		I2C2_DMABufTX[1] = 0x80;
		masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
		Delaynus(20000);
		I2C2_REENABLE = 1;
	}
	// Enable MPU
	MPU6000_Enable(ENABLE);
	// Enable I2C bypass to write to HMC5883
	MPU6000_EnableI2CBypass(ENABLE);
	if(I2C2_REENABLE)
	{
		// Reset slave devices
		// MPL3115
		I2C2_DMABufTX[0] = 38;
		I2C2_DMABufTX[1] = 0x04;
		masterSend(MPL3115A2_ADDRESS, I2C2_DMABufTX, 2);
		Delaynus(20000);
	}
	// Configure HMC5883
	HMC5883_Enable(ENABLE);
	// Configure MPL3115A2
	MPL3115A2_Enable(ENABLE);
	// Disable I2C bypass
	MPU6000_EnableI2CBypass(DISABLE);
	// Configure MPU I2C master mode
	MPU6000_ConfigureI2CMaster();
	// Enable MPU I2C master
	MPU6000_EnableI2CMaster(ENABLE);
	I2C2_INITDONE = 1;
}

// Copies sensor data from I2C to variables
void copySensorData(void)
{
	uint16_t uiTemp = 0;
	// Data is in I2C2_DMABufRX
	// Starts with reg 59, accel X high
	ACC_X = (I2C2_DMABufRX[0] << 8) & 0xff00;
	ACC_X = ACC_X | (I2C2_DMABufRX[1] & 0x00ff);
	ACC_Y = (I2C2_DMABufRX[2] << 8) & 0xff00;
	ACC_Y = ACC_Y | (I2C2_DMABufRX[3] & 0x00ff);
	ACC_Z = (I2C2_DMABufRX[4] << 8) & 0xff00;
	ACC_Z = ACC_Z | (I2C2_DMABufRX[5] & 0x00ff);
	// Reg 65,66(6,7) is IC temperature
	// Reg 67, 72 (8,13)is gyro
	GYRO_X = (I2C2_DMABufRX[8] << 8) & 0xff00;
	GYRO_X = GYRO_X | (I2C2_DMABufRX[9] & 0x00ff);
	GYRO_Y = (I2C2_DMABufRX[10] << 8) & 0xff00;
	GYRO_Y = GYRO_Y | (I2C2_DMABufRX[11] & 0x00ff);
	GYRO_Z = (I2C2_DMABufRX[12] << 8) & 0xff00;
	GYRO_Z = GYRO_Z | (I2C2_DMABufRX[13] & 0x00ff);
	// Reg 73,78 (14,19) is mag data
	MAG_X = (I2C2_DMABufRX[14] << 8) & 0xff00;
	MAG_X = MAG_X | (I2C2_DMABufRX[15] & 0x00ff);
	MAG_Y = (I2C2_DMABufRX[16] << 8) & 0xff00;
	MAG_Y = MAG_Y | (I2C2_DMABufRX[17] & 0x00ff);
	MAG_Z = (I2C2_DMABufRX[18] << 8) & 0xff00;
	MAG_Z = MAG_Z | (I2C2_DMABufRX[19] & 0x00ff);
	// Reg 79,80 (20,21) is barometer data
	BARO = (I2C2_DMABufRX[20] << 8) & 0xff00;
	BARO = BARO | (I2C2_DMABufRX[21] & 0x00ff);
	BARO = BARO * 10;
	uiTemp = I2C2_DMABufRX[22];
	uiTemp = uiTemp >> 4;
	uiTemp = uiTemp & 0x000F;
	BARO = BARO + uiTemp;
}

ErrorStatus MPU6000_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	masterReceive(MPU6000_ADDRESS, 107, I2C2_DMABufRX, 5);
	ui8RegState = I2C2_DMABufRX[0];

	I2C2_DMABufTX[0] = 107;
	if(newState == ENABLE)
	{
		// Set power = ON with clock source = X axis gyro
		ui8RegState = ui8RegState & ~_BIT6;
		I2C2_DMABufTX[1] = ui8RegState | 0x01;
	}
	else
	{
		// Set device to sleep
		I2C2_DMABufTX[1] = ui8RegState | 0x40;
	}
	masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
	Delaynus(2000);
	// Configure sensors
	// Register 26
	I2C2_DMABufTX[0] = 26;
	// Reg 26 = 0000 0010	Set low pass filter to 94 Hz
	I2C2_DMABufTX[1] = 0x02;
	// Reg 27 = 0001 0000	Set gyro maximum rate at 1000 °/sec
	I2C2_DMABufTX[2] = 0x10;
	// Reg 28 = 0001 0000	Set accel maximum rate at 8g
	I2C2_DMABufTX[3] = 0x10;
	masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 4);
	Delaynus(20000);

	return  error;
}

ErrorStatus MPU6000_EnableI2CBypass(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	masterReceive(MPU6000_ADDRESS, 55, I2C2_DMABufRX, 5);
	ui8RegState = I2C2_DMABufRX[0];

	I2C2_DMABufTX[0] = 55;
	if(newState == ENABLE)
	{
    	I2C2_DMABufTX[1] = ui8RegState | 0x02;
	}
	else
	{
		I2C2_DMABufTX[1] = ui8RegState & ~0x02;
	}
	masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
	Delaynus(20000);
	return  error;
}

ErrorStatus MPU6000_ConfigureI2CMaster(void)
{
	ErrorStatus error = ERROR;
	// Start with reg 36
	I2C2_DMABufTX[0] = 36;
	// I2C master control = 0101 1101; last four bits = I2C clock = 400 kHz
	I2C2_DMABufTX[1] = 0x5d;
	// Registers 37 - 39 - master 0 control
	// 37 - I2C_SLV0_ADDR = 1001 1110
	I2C2_DMABufTX[2] = 0x9E;
	// 38 - I2C_SLV0_REG - start read at this reg
	I2C2_DMABufTX[3] = 0x03;
	// 39 - I2C_SLV0_CTRL = 1000 0110
	I2C2_DMABufTX[4] = 0x86;
	// Registers 40 - 42 - master 1 control
	// 40 - I2C SLV1_ADDR = 1110 0000
	I2C2_DMABufTX[5] = 0xE0;
	// 41 = I2C_SLV1_REG - start read at this reg
	I2C2_DMABufTX[6] = 0x01;
	// 42 - I2C_SLV1_CTRL = 1000 0011
	I2C2_DMABufTX[7] = 0x83;
	masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 8);
	Delaynus(20000);
	return  error;
}

ErrorStatus MPU6000_EnableI2CMaster(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	masterReceive(MPU6000_ADDRESS, 106, I2C2_DMABufRX, 5);
	ui8RegState = I2C2_DMABufRX[0];

	I2C2_DMABufTX[0] = 106;
	if(newState == ENABLE)
	{
    	I2C2_DMABufTX[1] = ui8RegState | 0x20;
	}
	else
	{
		I2C2_DMABufTX[1] = ui8RegState & ~0x20;
	}
	masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
	Delaynus(20000);
	return  error;
}

ErrorStatus HMC5883_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	// Start at reg 0
	I2C2_DMABufTX[0] = 0;
	// CRA = 8 samples, 75 Hz, normal
	I2C2_DMABufTX[1] = 0x78;
	// CRB = +- 1,3 Gauss
	I2C2_DMABufTX[2] = 0x20;
	if(newState == ENABLE)
	{
		// Mode = continuous
		I2C2_DMABufTX[3] = 0x00;
	}
	else
	{
		// Mode = idle
		I2C2_DMABufTX[3] = 0x02;
	}
	masterSend(HMC5883_ADDRESS, I2C2_DMABufTX, 4);
	Delaynus(20000);
	return error;
}

ErrorStatus MPL3115A2_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	// Start at reg 38
	I2C2_DMABufTX[0] = 38;
	if(newState == ENABLE)
	{
		// Mode = enable, altimeter mode
		I2C2_DMABufTX[1] = 0x81;
	}
	else
	{
		// Mode = disable
		I2C2_DMABufTX[1] = 0x80;
	}
	//
	I2C2_DMABufTX[2] = 0x00;
	// Interrupt active high, open drain
	I2C2_DMABufTX[3] = 0x33;
	// Data ready interrupt enabled
	I2C2_DMABufTX[4] = 0x80;
	// Interrupt routed to pin 2
	I2C2_DMABufTX[5] = 0x00;

	masterSend(MPL3115A2_ADDRESS, I2C2_DMABufTX, 6);
	Delaynus(20000);
	return error;
}

ErrorStatus masterSend(uint8_t device, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;
	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);
	// Disable DMA TX Channel
	DMA_Cmd(DMA_I2C2_TX, DISABLE);
	// Wait until stream is disabled
	while (DMA_GetCmdStatus(DMA_I2C2_TX) != DISABLE)
	{
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_TX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_TX, &DMAInitStructure);

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_TX, DMA_FLAG_TCIF7 | DMA_FLAG_FEIF7 | DMA_FLAG_DMEIF7 |  DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{}
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// Transfer DMA data

	DMA_ClearFlag(DMA_I2C2_TX, DMA_FLAG_TCIF7);

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA TX Channel */
	DMA_Cmd(DMA_I2C2_TX, ENABLE);

	/* Wait until I2Cx_DMA_STREAM_RX enabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_TX)!= ENABLE)
	{
		// Check if we have complete interrupt
		if(DMA_GetFlagStatus(DMA_I2C2_TX,DMA_FLAG_TCIF7)!=RESET)
		{
			// If yes, break
			//Delaynus(2000);
			break;
		}
	}

	/* Transfer complete or time out */
	while (DMA_GetFlagStatus(DMA_I2C2_TX,DMA_FLAG_TCIF7)==RESET)
	{
	}

	// Check TxE bit
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
	{
	}

	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_TX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_TX)!= DISABLE)
	{}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	return SUCCESS;
}

// Function starts DMA receive process
ErrorStatus masterReceive_beginDMA(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;

	// Mark receive in progress
	I2C2_WAITINGDATA = 1;
	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Wait until stream is disabled
	while (DMA_GetCmdStatus(DMA_I2C2_RX) != DISABLE)
	{
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_RX, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{}
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, startReg);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send I2Cx START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2Cx EV5 and clear it or time out*/

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{}

	/* Send I2Cx slave Address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on I2Cx EV6 and clear it or time out */

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{}

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, ENABLE);

	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);
	return ERROR;
}

ErrorStatus masterReceive(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;

	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Wait until stream is disabled
	while (DMA_GetCmdStatus(DMA_I2C2_RX) != DISABLE)
	{
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_RX, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{}
	}

	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, startReg);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send I2Cx START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2Cx EV5 and clear it or time out*/

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{}

	/* Send I2Cx slave Address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on I2Cx EV6 and clear it or time out */

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{}

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, ENABLE);

	/* Wait until I2Cx_DMA_STREAM_RX enabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= ENABLE)
	{}

	/* Transfer complete or time out */

	while (DMA_GetFlagStatus(DMA_I2C2_RX,DMA_FLAG_TCIF3)==RESET)
	{
	}

	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= DISABLE)
	{}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	return SUCCESS;
}

ErrorStatus masterReceive_HMC5883L(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;

	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Wait until stream is disabled
	while (DMA_GetCmdStatus(DMA_I2C2_RX) != DISABLE)
	{
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_RX, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{}
	}

	/* Send I2Cx START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2Cx EV5 and clear it or time out*/

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{}

	/* Send I2Cx slave Address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on I2Cx EV6 and clear it or time out */

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{}

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, ENABLE);

	/* Wait until I2Cx_DMA_STREAM_RX enabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= ENABLE)
	{}

	/* Transfer complete or time out */

	while (DMA_GetFlagStatus(DMA_I2C2_RX,DMA_FLAG_TCIF3)==RESET)
	{
	}

	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= DISABLE)
	{}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	return SUCCESS;
}
