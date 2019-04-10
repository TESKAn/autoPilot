/*
 * sensors_spi.c
 *
 *  Created on: 10. apr. 2019
 *      Author: jmoc
 */

#include "allinclude.h"

int16_t Sensor_SPIInit()
{

	return 0;
}

int16_t Sensor_SPIWrite()
{

	return 0;
}

int16_t Sensor_SPIRead()
{

	return 0;
}

int16_t Sensor_SPIReadDMA(uint8_t device)
{
	DMA_InitTypeDef DMAInitStructure;
	uint16_t ui16Temp = 0;
	uint32_t byteCount = 0;
	uint16_t ui16StartRegAddress = 0;

	// Mark receive in progress
	SPI3_WAITINGDATA = 1;
	// Disable I2C2 interrupts
	SPI_ITConfig(SPI2, SPI_I2S_IT_TXE | SPI_I2S_IT_RXNE | SPI_I2S_IT_ERR, DISABLE);

	// Disable DMA RX stream
	DMA_Cmd(DMA_SPI3_RX_STREAM, DISABLE);
	// Wait until stream is disabled
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_SPI3_RX_STREAM) != DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > SPI_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
	}

	// Set correct CS bit
	switch(SPI_SensorBuf.ui8Device)
	{
		case GYRO_DEV_CS:
		{
			// A/G module
			A_G_CS_0;
			byteCount = GYRO_BYTE_COUNT;
			ui16StartRegAddress = GYRO_START_REG;
			break;
		}
		case ACC_DEV_CS:
		{
			A_G_CS_0;
			byteCount = ACC_BYTE_COUNT;
			ui16StartRegAddress = ACC_START_REG;
			break;
		}
		case MAG_DEV_CS:
		{
			MAG_CS_0;
			byteCount = MAG_BYTE_COUNT;
			ui16StartRegAddress = MAG_START_REG;
			break;
		}

	}

	// Deinit DMA
	DMA_DeInit(DMA_SPI3_RX_STREAM);
	// Configure SPI DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_SPI3_RX_CHANNEL;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)(&SPI_SensorBuf.DATA.buf);
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
	DMA_Init(DMA_SPI3_RX_STREAM, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_SPI3_RX_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	// Start SPI RX process
	ui16Temp = 0x80 + ui16StartRegAddress;
	SPI_I2S_SendData(SPI3, ui16Temp);

	/* SPI DMA Enable */
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_SPI3_RX_STREAM, ENABLE);

	DMA_ITConfig(DMA_SPI3_RX_STREAM, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);

	return 0;
}

int16_t Sensor_SPIInitAG()
{
	switch(i16SPIInitAGState)
	{
		case 0:
		{
			i16SPIInitAGState++;
			break;
		}
		default:
		{
			i16SPIInitAGState = 0;
		}
	}

	return 0;
}
int16_t Sensor_SPIInitM()
{

	return 0;
}
int16_t Sensor_SPIInitB()
{

	return 0;
}
