/*
 * RS485comm.c
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */


#include "allinclude.h"

#define SERVO_FR_ID		0x01
#define SERVO_FL_ID		0x02
#define SERVO_R_ID		0x03

RS485SERVO RS485Servo_FL;
RS485SERVO RS485Servo_FR;
RS485SERVO RS485Servo_R;

UInt8 RS485TransmittBuffer[64];

UInt8 RS485BytesToReceive = 0;

UInt8 RS485MasterState = RS485_M_IDLE;

UInt16 RS485TimerCount = 0;
UInt16 RS485TimerTimeout = 5000;


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

// Master functions
UInt16 RS485_MasterInitData(void)
{
	Int16 i16Temp = 0;
	// Set slaves
	RS485Servo_FL.REGS.ui8ID = SERVO_FR_ID;
	RS485Servo_FR.REGS.ui8ID = SERVO_FL_ID;
	RS485Servo_R.REGS.ui8ID = SERVO_R_ID;

	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	return 0;
}

UInt16 RS485_MasterWriteByte(void)
{

	// Transmit data

	// Enable transmit
	RS485_TXEN;
	// Send
	//transferDMA_USART1(uint8_t *data, int length)

	return 0;
}


UInt16 RS485_States_Master(void)
{
	Int16 i = 0;
	switch(RS485MasterState)
	{
		case RS485_M_STATE_IDLE:
		{
			break;
		}
		case RS485_M_STATE_WRITE:
		{
			break;
		}

		case RS485_M_STATE_TORQUE_ON:
		{

			break;
		}
		case RS485_M_STATE_TORQUE_OFF:
		{

			break;
		}
		case RS485_M_STATE_READ_ALL:
		{
			break;
		}
		case RS485_M_STATE_SET_POS:
		{
			break;
		}
		case RS485_M_STATE_SET_SPEED:
		{

			break;
		}
		case RS485_M_STATE_SET_COMPLIANCE:
		{

			break;
		}
		case RS485_M_STATE_REQUEST:
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


UInt16 RS485_decodeMessage(void)
{
	UInt8 ui8Temp = 0;

	return 0;
}

