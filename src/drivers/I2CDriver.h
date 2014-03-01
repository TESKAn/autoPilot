/*
 * I2CDriver.h
 *
 *  Created on: 1. mar. 2014
 *      Author: Jure
 */

#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

// Define buffer size in bytes
#define I2C_BUFFER_SIZE		128

// Data structure for I2C
typedef struct
{
	uint32_t flag;
	// Peripheral to use
	I2C_TypeDef* I2C;
	// State machine state
	uint8_t I2CState;
	// Interrupt state machine
	uint8_t I2CInterruptState;
	// Selected slave address
	uint8_t slaveID;
	// Slave start register
	uint8_t startReg;
	// Data buffer
	uint8_t buffer[I2C_BUFFER_SIZE];
	// Bytes to send
	uint16_t bytesToSend;
	// Bytes to receive
	uint16_t bytesToReceive;
	// DMA to use with I2C
	// Channel
	uint32_t DMARXChannel;
	uint32_t DMATXChannel;
	// Stream
	uint32_t DMARXStream;
	uint32_t DMATXStream;
	// DMA interrupt mask
	uint32_t DMARXInterruptMask;
	uint32_t DMATXInterruptMask;
	// DMA flag mask
	uint32_t DMARXFlagMask;
	uint32_t DMATXFlagMask;


}I2CDriver;


// Function declaration
void I2C_Read(I2CDriver* data);
void I2C_Write(I2CDriver* data);
uint8_t I2C_CheckStatus(I2CDriver* data);
void I2C_EV_ISRHandler(I2CDriver* data);
void I2C_ER_ISRHandler(I2CDriver* data);


#endif /* I2CDRIVER_H_ */
