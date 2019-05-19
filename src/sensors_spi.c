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


// Call every 1 ms, manage all SPI traffic.
int16_t Sensor_SPICommProcess()
{
	uint32_t ui32Temp = 0;
	uint16_t ui16Temp = 0;
	uint16_t ui16Address = 0;
	uint16_t ui16AddressOffset = 0;
	uint8_t ui8Temp = 0;
	// Is SPI idle?
	if(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY))
	{
		if(0 < rb32SensorTXQueue.count)
		{
			ui32Temp = RB32_pop(&rb32SensorTXQueue);
			switch(ui32Temp)
			{
				case 0:
				{
					break;
				}
				case ACC_GET_DATA:
				{
					// Store buffer address
					SPI_SensorBuf = &SPI_SensorBufAcc;
					SPI_SensorBuf->ui8Device = ACC_DEV_CS;
					// Read data from mag
					Sensor_SPIReadDMA();
					break;
				}
				case GYRO_GET_DATA:
				{
					// Store buffer address
					SPI_SensorBuf = &SPI_SensorBufGyro;
					SPI_SensorBuf->ui8Device = GYRO_DEV_CS;
					// Read data from mag
					Sensor_SPIReadDMA();
					break;
				}
				case MAG_GET_DATA:
				{
					// Store buffer address
					SPI_SensorBuf = &SPI_SensorBufMag;
					SPI_SensorBuf->ui8Device = MAG_DEV_CS;
					// Read data from mag
					Sensor_SPIReadDMA();

					break;
				}
				case BARO_GET_DATA:
				{
					// Store buffer address
					SPI_SensorBuf = &SPI_SensorBufBaro;
					SPI_SensorBuf->ui8Device = BARO_DEV_CS;
					// Read data from mag
					Sensor_SPIReadDMA();
					break;
				}
				case SENSOR_WRITE_REG:
				{
					i16SPITestData = Sensor_SPIWrite(ui16SPITestDevice, ui16SPITestAddress, i16SPITestData);
					break;
				}
				case SENSOR_READ_REG:
				{
					i16SPITestData = Sensor_SPIRead(ui16SPITestDevice, ui16SPITestAddress);
					break;
				}
				case SENSOR_READ_WORD:
				{
					i16SPITestData = Sensor_SPIReadWord(ui16SPITestDevice, ui16SPITestAddress);
					break;
				}
				case SENSOR_READ_T:
				{

					break;
				}
				default:
				{
					break;
				}
			}
		}
	}
	return 0;
}

int16_t Sensor_setCS(uint8_t device, uint8_t state)
{
	switch(device)
	{
		case GYRO_DEV_CS:
		case ACC_DEV_CS:
		{
			if(state)
			{
				A_G_CS_1;
			}
			else
			{
				A_G_CS_0;
			}
			break;
		}
		case MAG_DEV_CS:
		{
			if(state)
			{
				MAG_CS_1;
			}
			else
			{
				MAG_CS_0;
			}
			break;
		}
		case BARO_DEV_CS:
		{
			if(state)
			{
				BARO_CS_1;
			}
			else
			{
				BARO_CS_0;
			}
			break;
		}
	}
	return 0;
}

int16_t Sensor_SPIWrite(uint8_t device, uint8_t address, uint8_t data)
{
	uint16_t ui16Temp = 0;
	// Is SPI idle?
	if(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY))
	{
		// Cleanup SPI
		// Wait until TX is empty
		while(!(SPI3->SR & SPI_I2S_FLAG_TXE)){}
		// Check that there is no data in RX buffer
		if(SPI3->SR & SPI_I2S_FLAG_RXNE)
		{
			ui16Temp = SPI3->DR;
		}

		ui16Temp = 0x7F & (uint16_t)address;

		// Set CS low
		Sensor_setCS(device, 0);

		SPI3->DR = ui16Temp;
		// Wait until RX is ready
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		// Send data
		SPI3->DR = (uint16_t)data;
		// Wait until RX buffer is full
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		Sensor_setCS(device, 1);
	}
	return 0;
}

int16_t Sensor_SPIRead(uint8_t device, uint8_t address)
{
	uint16_t ui16Temp = 0;
	// Is SPI idle?
	if(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY))
	{
		// Cleanup SPI
		// Wait until TX is empty
		while(!(SPI3->SR & SPI_I2S_FLAG_TXE)){}
		// Check that there is no data in RX buffer
		if(SPI3->SR & SPI_I2S_FLAG_RXNE)
		{
			ui16Temp = SPI3->DR;
		}

		ui16Temp = 0x80 | (uint16_t)address;

		// Set CS low
		Sensor_setCS(device, 0);

		// Send address
		SPI3->DR = ui16Temp;
		// Wait until TX is empty
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		// Send data
		SPI3->DR = 0xff;
		// Wait until RX buffer is full
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		Sensor_setCS(device, 1);
	}
	return (int16_t)ui16Temp;
}

int16_t Sensor_SPIReadWord(uint8_t device, uint8_t address)
{
	uint16_t ui16Temp = 0;
	uint16_t ui16Result = 0;
	// Is SPI idle?
	if(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY))
	{
		// Cleanup SPI
		// Wait until TX is empty
		while(!(SPI3->SR & SPI_I2S_FLAG_TXE)){}
		// Check that there is no data in RX buffer
		if(SPI3->SR & SPI_I2S_FLAG_RXNE)
		{
			ui16Temp = SPI3->DR;
		}

		ui16Temp = 0x80 | (uint16_t)address;

		// Set CS low
		Sensor_setCS(device, 0);

		// Send address
		SPI3->DR = ui16Temp;
		// Wait until TX is empty
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		// Send data
		SPI3->DR = 0xff;
		// Wait until RX buffer is full
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Result = SPI3->DR;

		// Send data
		SPI3->DR = 0xff;
		// Wait until RX buffer is full
		while(!(SPI3->SR & SPI_I2S_FLAG_RXNE))
		{

		}
		// Read data
		ui16Temp = SPI3->DR;

		Sensor_setCS(device, 1);

		ui16Temp = ui16Temp << 8;
		ui16Temp = ui16Temp & 0xff00;
		ui16Result = ui16Result & 0xff;
		ui16Result = ui16Result | ui16Temp;

	}
	return (int16_t)ui16Result;
}

int16_t Sensor_SPIReadDMA()
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

	// Disable DMA TX stream
	DMA_Cmd(DMA_SPI3_TX_STREAM, DISABLE);
	// Wait until stream is disabled
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_SPI3_TX_STREAM) != DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > SPI_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
	}

	byteCount = SPI_SensorBuf->ui32ByteCount;
	ui16StartRegAddress = SPI_SensorBuf->ui16StartReg;

	// Set dummy byte
	SPI_SensorBuf->ui8DummyByte = 0xff;

	// Setup DMA RX stream
	// Deinit DMA
	DMA_DeInit(DMA_SPI3_RX_STREAM);
	// Configure SPI DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_SPI3_RX_CHANNEL;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)(&SPI_SensorBuf->DATA.buf[0]);
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

	// Clear DMA flags
	DMA_ClearFlag(DMA_SPI3_RX_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Master Receiver -----------------------------------------------------------*/

	// Setup DMA TX stream
	// Deinit DMA
	DMA_DeInit(DMA_SPI3_TX_STREAM);
	// Configure SPI DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_SPI3_TX_CHANNEL;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)(&SPI_SensorBuf->ui8DummyByte);
	//DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)(&SPI_SensorBuf->ui8TxBuf[0]);
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_SPI3_TX_STREAM, &DMAInitStructure);

	// Clear DMA flags
	DMA_ClearFlag(DMA_SPI3_TX_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	// Cleanup SPI
	// Wait until TX is empty
	while(!(SPI3->SR & SPI_I2S_FLAG_TXE)){}
	// Check that there is no data in RX buffer
	if(SPI3->SR & SPI_I2S_FLAG_RXNE)
	{
		ui16Temp = SPI3->DR;
	}

	//SPI_Cmd(SPI3, DISABLE);
	// Start SPI RX process
	// Address
	ui16Temp = 0x80 | ui16StartRegAddress;

	//SPI_SensorBuf->ui8TxBuf[0] = ui16Temp;

	// Store to send buffer
	SPI_SensorBuf->ui8DummyByte = ui16Temp;
	// Set correct CS bit
	switch(SPI_SensorBuf->ui8Device)
	{
		case GYRO_DEV_CS:
		{
			// A/G module
			A_G_CS_0;
			break;
		}
		case ACC_DEV_CS:
		{
			A_G_CS_0;
			break;
		}
		case MAG_DEV_CS:
		{
			MAG_CS_0;
			break;
		}
		case BARO_DEV_CS:
		{
			BARO_CS_0;
			break;
		}
	}

	//SPI_I2S_SendData(SPI3, ui16Temp);

	/* Enable DMA RX, TX Channel */
	DMA_Cmd(DMA_SPI3_TX_STREAM, ENABLE);
	DMA_Cmd(DMA_SPI3_RX_STREAM, ENABLE);

	/* SPI DMA Enable */
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);

	//SPI_Cmd(SPI3, ENABLE);

	DMA_ITConfig(DMA_SPI3_RX_STREAM, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);
	DMA_ITConfig(DMA_SPI3_TX_STREAM, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);

	return 0;
}

// LSM9DS1
int16_t Sensor_SPIInitAG()
{
	Sensor_SPIWrite(1, 0x10, A_G_CTRL_REG1_G);
	Sensor_SPIWrite(1, 0x11, A_G_CTRL_REG2_G);
	Sensor_SPIWrite(1, 0x12, A_G_CTRL_REG3_G);

	Sensor_SPIWrite(1, 0x1e, A_G_CTRL_REG4_G);
	Sensor_SPIWrite(1, 0x1f, A_G_CTRL_REG5_XL);
	Sensor_SPIWrite(1, 0x20, A_G_CTRL_REG6_XL);
	Sensor_SPIWrite(1, 0x21, A_G_CTRL_REG7_XL);
	Sensor_SPIWrite(1, 0x22, A_G_CTRL_REG8);
	Sensor_SPIWrite(1, 0x23, A_G_CTRL_REG9);

	Sensor_SPIWrite(1, 0x0c, A_G_INT1_CTRL);
	Sensor_SPIWrite(1, 0x0d, A_G_INT2_CTRL);

	Sensor_SPIWrite(1, 0x30, A_G_INT_GEN_CFG_G);

	return 0;
}


int16_t Sensor_SPIInitM()
{
	Sensor_SPIWrite(3, 0x20, MAG_CTRL_REG_1_M);
	Sensor_SPIWrite(3, 0x21, MAG_CTRL_REG_2_M);
	Sensor_SPIWrite(3, 0x22, MAG_CTRL_REG_3_M);
	Sensor_SPIWrite(3, 0x23, MAG_CTRL_REG_4_M);
	Sensor_SPIWrite(3, 0x24, MAG_CTRL_REG_5_M);

	Sensor_SPIWrite(3, 0x30, MAG_INT_CFG_M);

	return 0;
}
int16_t Sensor_SPIInitB()
{
	Sensor_SPIWrite(4, 0x20, BARO_CTRL_REG1);
	Sensor_SPIWrite(4, 0x21, BARO_CTRL_REG2);
	Sensor_SPIWrite(4, 0x22, BARO_CTRL_REG3);
	Sensor_SPIWrite(4, 0x23, BARO_CTRL_REG4);

	return 0;
}
