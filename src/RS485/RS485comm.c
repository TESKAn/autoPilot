/*
 * RS485comm.c
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */


#include "allinclude.h"

// Default defines
#define SERVO_FR_ID		0x01
#define SERVO_FL_ID		0x02
#define SERVO_R_ID		0x03

UInt8 servoFRID = SERVO_FR_ID;
UInt8 servoFLID = SERVO_FL_ID;
UInt8 servoRID = SERVO_R_ID;

RS485SERVO RS485Servo_FL;
RS485SERVO RS485Servo_FR;
RS485SERVO RS485Servo_R;

UInt8 RS485TransmittBuffer[64];
UInt8 RS485RXBuffer[128];

UInt8 RS485BytesToReceive = 0;

UInt8 RS485MasterState = RS485_M_IDLE;
UInt8 RS485MasterPoll = 0;

UInt16 RS485TimerCount = 0;
UInt16 RS485TimerTimeout = 5000;

UInt8 RS485ReceiveState = 0;
UInt8 RS485RXBufIndex = 0;
UInt8 RS485RXBytesLeft = 0;
UInt8 RS485Checksum = 0;

// Variables
uint8_t RS485_CommandBuffer[16];

RING_BUFFER RS485CommandBuffer;

// Master functions
UInt16 RS485_MasterInitData(void)
{
	Int16 i16Temp = 0;
	// Set slaves
	RS485Servo_FL.REGS.ui8ID = servoFRID;
	RS485Servo_FR.REGS.ui8ID = servoFLID;
	RS485Servo_R.REGS.ui8ID = servoRID;

	// Init command buffer
	RB_Init(&RS485CommandBuffer, RS485_CommandBuffer, 16);

	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	return 0;
}


UInt16 RS485_ServoEnable(UInt8 servoID)
{

	return 0;
}

UInt16 RS485_ServoTorqueON(UInt8 servoID)
{
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x18;					// Reg address 24
	RS485TransmittBuffer[6] = 0x01;					// Data to write
	RS485TransmittBuffer[7] = ~(0x20 + servoID);	// Checksum
	return 8;
}

UInt16 RS485_ServoTorqueOFF(UInt8 servoID)
{
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x18;					// Reg address 24
	RS485TransmittBuffer[6] = 0x00;					// Data to write
	RS485TransmittBuffer[7] = ~(0x20 + servoID);	// Checksum
	return 8;
}

UInt16 RS485_ServoReadAll(UInt8 servoID)
{
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_READ;	// Command
	RS485TransmittBuffer[5] = 0x00;					// Reg address 0
	RS485TransmittBuffer[6] = 0x49;					// Data to read - 73 regs
	RS485TransmittBuffer[7] = ~(0x4f + servoID);	// Checksum
	return 8;
}

UInt16 RS485_ServoSetPosition(UInt8 servoID, UInt16 servoPosition)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x05;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x1E;					// Reg address - goal position
	RS485TransmittBuffer[6] = servoPosition & 0xFF;
	RS485TransmittBuffer[7] = (servoPosition >> 8) & 0xFF;

	// Calculate checksum
	RS485TransmittBuffer[8] = RS485TransmittBuffer[2];
	for(i = 3; i < 8; i++)
	{
		RS485TransmittBuffer[8] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[8] = ~(RS485TransmittBuffer[8]);				// Checksum
	return 9;
}

UInt16 RS485_ServoSetSpeed(UInt8 servoID, UInt16 servoSpeed)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x05;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x20;					// Reg address - goal position
	RS485TransmittBuffer[6] = servoSpeed & 0xFF;
	RS485TransmittBuffer[7] = (servoSpeed >> 8) & 0xFF;

	// Calculate checksum
	RS485TransmittBuffer[8] = RS485TransmittBuffer[2];
	for(i = 3; i < 8; i++)
	{
		RS485TransmittBuffer[8] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[8] = ~(RS485TransmittBuffer[8]);				// Checksum
	return 9;
}

UInt16 RS485_ServoSetCompliance(UInt8 servoID, UInt8 CWMargin, UInt8 CCWMargin, UInt8 CWSlope, UInt8 CCWSlope)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x05;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x26;					// Reg address - compliance
	RS485TransmittBuffer[6] = CWMargin;
	RS485TransmittBuffer[7] = CCWMargin;
	RS485TransmittBuffer[8] = CWSlope;
	RS485TransmittBuffer[9] = CCWSlope;

	// Calculate checksum
	RS485TransmittBuffer[10] = RS485TransmittBuffer[2];
	for(i = 3; i < 10; i++)
	{
		RS485TransmittBuffer[10] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[10] = ~(RS485TransmittBuffer[10]);				// Checksum
	return 11;
}

UInt16 RS485_BufferQueuedCommand(UInt8 command)
{
	UInt16 bytes = 0;
	switch(command)
	{
		case RS485_SERVO_FR_TORQ_ON:
		{
			bytes = RS485_ServoTorqueON(servoFRID);
			break;
		}
		case RS485_SERVO_FR_TORQ_OFF:
		{
			bytes = RS485_ServoTorqueOFF(servoFRID);
			break;
		}
		case RS485_SERVO_FL_TORQ_ON:
		{
			bytes = RS485_ServoTorqueON(servoFLID);
			break;
		}
		case RS485_SERVO_FL_TORQ_OFF:
		{
			bytes = RS485_ServoTorqueOFF(servoFLID);
			break;
		}
	}
	return bytes;
}


UInt16 RS485_MasterWriteByte(uint8_t *data, int length)
{

	// Transmit data

	// Enable transmit
	RS485_TXEN;
	// Send
	transferDMA_USART1(data, length);

	return 0;
}


UInt16 RS485_States_Master(void)
{
	UInt8 command = 0;
	UInt16 bytesToSend = 0;
	switch(RS485MasterState)
	{
		case RS485_M_STATE_IDLE:
		{
			break;
		}
		case RS485_M_STATE_POLL:
		{
			// Check send buffer
			if(0 == RS485CommandBuffer.count)
			{

			}
			else
			{
				// Get command from buffer
				command = RB_pop(&RS485CommandBuffer);
				// Store req bytes in tx buffer
				bytesToSend = RS485_BufferQueuedCommand(command);
				// Send data with DMA
				RS485_MasterWriteByte(&RS485TransmittBuffer, bytesToSend);

			}
			break;
		}

		case RS485_M_STATE_WAITING_RESPONSE:
		{

			break;
		}

		default:
		{

			break;
		}
	}
	return 0;
}

UInt16 RS485_ReceiveMessage(UInt8 data)
{
	switch(RS485ReceiveState)
	{
		case RS485_M_IDLE:
		{
			// Do we have 0xff?
			if(0xff == data)
			{
				RS485ReceiveState = RS485_M_WAIT_FOR_SIGNAL;
			}
			break;
		}
		case RS485_M_WAIT_FOR_SIGNAL:
		{
			// Do we have 0xff?
			if(0xff == data)
			{
				RS485ReceiveState = RS485_M_WAIT_FOR_ID;
			}
			break;
		}
		case RS485_M_WAIT_FOR_ID:
		{
			// Checksum
			RS485Checksum = data;
			// Store ID
			RS485RXBuffer[0] = data;
			RS485ReceiveState = RS485_M_WAIT_FOR_LENGTH;
			break;
		}
		case RS485_M_WAIT_FOR_LENGTH:
		{
			// Checksum
			RS485Checksum += data;
			// Store length
			RS485RXBuffer[1] = data;
			RS485RXBytesLeft = data;
			RS485RXBufIndex = 2;
			RS485ReceiveState = RS485_M_WAIT_FOR_DATA;
			break;
		}
		case RS485_M_WAIT_FOR_DATA:
		{
			// Checksum
			RS485Checksum += data;
			// Store received byte
			RS485RXBuffer[RS485RXBufIndex] = data;
			RS485RXBufIndex++;
			RS485RXBytesLeft--;
			if(0 == RS485RXBytesLeft)
			{
				RS485ReceiveState = RS485_M_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_M_WAIT_FOR_CHECKSUM:
		{
			RS485Checksum = ~RS485Checksum;
			RS485RXBuffer[RS485RXBufIndex] = data;
			if(RS485Checksum == data)
			{
				// Data valid, decode
				RS485_DecodeMessage();
			}
			RS485ReceiveState = RS485_M_IDLE;
			RS485RXBytesLeft = 0;
			RS485RXBufIndex = 0;
			break;
		}
		default:
		{
			RS485ReceiveState = RS485_M_IDLE;
			RS485RXBytesLeft = 0;
			RS485RXBufIndex = 0;
			break;
		}
	}
	return 0;
}

UInt16 RS485_DecodeMessage()
{
	RS485SERVO *SelectedServo;
	// Check ID
	switch(RS485RXBuffer[0])
	{
		case servoFRID:
		{
			SelectedServo = RS485Servo_FL;
			break;
		}
	}
	return 0;
}
