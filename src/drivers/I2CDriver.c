/*
 * I2CDriver.c
 *
 *  Created on: 1. mar. 2014
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "I2CDriver.h"

// Macros used by driver

// Flag masks
#define MASTER_R			0X0001
#define MASTER_WRITE		0X0002
#define MASTER_READ			0X0004
#define MASTER_START_OK		0X0008
#define MASTER_ADDRESS_OK	0X0010

// State machine
#define I2C_IDLE				0
#define I2C_IN_PROGRESS			1

// Interrupt state machine
#define I2CI_FIRSTSTART			0
#define I2CI_ADDRESS_WRITE		1
#define I2CI_STARTREG			2
#define I2CI_REPEATEDSTART		3
#define I2CI_ADDRESS_READ		4

// Read data
void I2C_Read(I2CDriver* data)
{
	if(I2C_IDLE == data->I2CState)
	{
		// Write data from buffer to device
		data->flag |= MASTER_R;
		// Set interrupt state
		data->I2CInterruptState = I2CI_FIRSTSTART;
		// Send START
		I2C_GenerateSTART(data->I2C, ENABLE);
		// State to "IN PROGRESS"
		data->I2CState = I2C_IN_PROGRESS;
	}
}

// Write data
void I2C_Write(I2CDriver* data)
{
	if(I2C_IDLE == data->I2CState)
	{
		// Write data from buffer to device
		data->flag &=  ~MASTER_R;
		// Set interrupt state
		data->I2CInterruptState = I2CI_FIRSTSTART;
		// Send START
		I2C_GenerateSTART(data->I2C, ENABLE);
		// State to "IN PROGRESS"
		data->I2CState = I2C_IN_PROGRESS;
	}
}

// Check if I2C operation is finished
uint8_t I2C_CheckStatus(I2CDriver* data)
{
	if(I2C_IDLE == data->I2CState)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


// I2C event ISR
void I2C_EV_ISRHandler(I2CDriver* data)
{
	uint32_t event = 0;
	// Get event that triggered interrupt
	event = I2C_GetLastEvent(data->I2C);
	// Go through states



	switch(data->I2CInterruptState)
	{
		case I2CI_FIRSTSTART:
		{
			// Check if start event
			if(I2C_EVENT_MASTER_MODE_SELECT == event)
			{
				// Start successfully sent
				// Send address - write
				I2C_Send7bitAddress(data->I2C, data->slaveID, I2C_Direction_Transmitter);
				data->I2CInterruptState = I2CI_ADDRESS_WRITE;
			}
			break;
		}
		case I2CI_ADDRESS_WRITE:
		{
			// Check if transmitter mode has been selected
			if(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED == event)
			{
				// Send register address
				I2C_SendData(data->I2C, data->startReg);
				data->I2CInterruptState = I2CI_STARTREG;
			}
			break;
		}
		case I2CI_STARTREG:
		{
			// If byte has been transmitted
			if(I2C_EVENT_MASTER_BYTE_TRANSMITTED == event)
			{
				// If reading
				if(data->flag && MASTER_R)
				{
					// Send repeated start
					I2C_GenerateSTART(data->I2C, ENABLE);
					data->I2CInterruptState = I2CI_REPEATEDSTART;
				}
				// Else if writing
				else
				{
					// Enable DMA stream write
				}
			}

			break;
		}
		case I2CI_REPEATEDSTART:
		{
			// If repeated start successfull
			if(I2C_EVENT_MASTER_MODE_SELECT == event)
			{
				// Send slave address for read
				I2C_Send7bitAddress(data->I2C, data->slaveID, I2C_Direction_Receiver);
				data->I2CInterruptState = I2CI_ADDRESS_READ;
			}
			break;
		}
		case I2CI_ADDRESS_READ:
		{
			// Setup DMA for read
		}
	}
}

// I2C error ISR
void I2C_ER_ISRHandler(I2CDriver* data)
{

}

// DMA TX stream ISR
void DMA_TXStream_ISRHandler(I2CDriver* data)
{
	DMA_ClearFlag(data->DMATXStream, data->DMATXFlagMask);
	DMA_ClearITPendingBit(data->DMATXStream, data->DMATXInterruptMask);
}

// DMA RX stream ISR
void DMA_RXStream_ISRHandler(I2CDriver* data)
{
	DMA_ClearFlag(data->DMARXStream, data->DMARXFlagMask);
	DMA_ClearITPendingBit(data->DMARXStream, data->DMARXInterruptMask);
}
