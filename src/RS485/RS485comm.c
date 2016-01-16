/*
 * RS485comm.c
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */


#include "allinclude.h"

// Default defines
#define SERVO_FR_ID		0x13
#define SERVO_FL_ID		0x12
#define SERVO_R_ID		0x11
#define MOTOR_FR_ID		0x23
#define MOTOR_FL_ID		0x22
#define MOTOR_R_ID		0x21

UInt8 servoFRID = SERVO_FR_ID;
UInt8 servoFLID = SERVO_FL_ID;
UInt8 servoRID = SERVO_R_ID;

UInt8 motorFRID = MOTOR_FR_ID;
UInt8 motorFLID = MOTOR_FL_ID;
UInt8 motorRID = MOTOR_R_ID;

Int8 RS485PollInterval = 10;

RS485SERVO RS485Servo_FL;
RS485SERVO RS485Servo_FR;
RS485SERVO RS485Servo_R;

RS485MOTOR RS485Motor_FL;
RS485MOTOR RS485Motor_FR;
RS485MOTOR RS485Motor_R;

UInt8 RS485TransmittBuffer[64];
UInt8 RS485RXBuffer[128];

UInt8 RS485BytesToReceive = 0;

UInt8 RS485MasterState = RS485_M_IDLE;
UInt8 RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
UInt8 RS485MasterPoll = 0;

// 20 ms timeout
UInt16 RS485TimerCount = 0;
UInt16 RS485TimerTimeout = 500;
UInt8 RS485ResponseReceived = 0;

UInt8 RS485ReceiveState = 0;
UInt8 RS485RXBufIndex = 0;
UInt8 RS485RXBytesLeft = 0;
UInt8 RS485Checksum = 0;
UInt8 RS485ReadReqStartAddress = 0;

// Variables
UInt32 RS485_CommandBuffer[16];

RING_BUFFER32 RS485CommandBuffer;

RS485COMMAND RS485CommandDecoder;

CONVERTNUM RS485ConvertNum;

Int16 i16RS485Timing = 0;

Int16 i16RS485TimeOut = 100;


Int16 RS485_Timing()
{
	i16RS485Timing++;
	if(100 <= i16RS485Timing)
	{
		i16RS485Timing = 0;
		UART_QueueMessagei16(VAR_SERVOFL, RS485Servo_FL.REGS.ui16PresentPosition);
		UART_QueueMessagei16(VAR_SERVOFR, RS485Servo_FR.REGS.ui16PresentPosition);
		UART_QueueMessagei16(VAR_SERVOR, RS485Servo_R.REGS.ui16PresentPosition);
	}
	// Check RX process
	if(RS485_M_IDLE != RS485ReceiveState)
	{
		i16RS485TimeOut--;
		if(0 >= i16RS485TimeOut)
		{
			RS485ReceiveState = RS485_M_IDLE;
			i16RS485TimeOut = 100;
			RS485RXBytesLeft = 0;
			RS485RXBufIndex = 0;
		}
	}
	return 0;
}


// Master functions
UInt16 RS485_MasterInitData(void)
{
	// Set slaves
	RS485Servo_FL.REGS.ui8ID = servoFRID;
	RS485Servo_FR.REGS.ui8ID = servoFLID;
	RS485Servo_R.REGS.ui8ID = servoRID;

	// Init command buffer
	RB32_Init(&RS485CommandBuffer, RS485_CommandBuffer, 16);

	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	return 0;
}

UInt16 RS485_MasterState(int state)
{
	switch(state)
	{
		case 1:
		{
			RS485MasterState = RS485_M_STATE_POLL;
			break;
		}
		case 2:
		{
			RS485MasterState = RS485_M_STATE_IDLE;
			break;
		}
	}
	return 0;
}

Int16 RS485_MotorTest(UInt8 func)
{
	RS485COMMAND motorCommand;
	switch(func)
	{
		case 0:
		{
			// Motor 1
			motorCommand.VARS.ui8Address = motorFRID;
			motorCommand.VARS.ui8Command = RS485_SET_MOTOR_POSITION;
			motorCommand.VARS.ui16Data = servoMovePosition;
			// Store command
			RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
			break;
		}
		case 1:
		{
			// Arm motor - run
			motorCommand.VARS.ui8Address = motorFRID;
			motorCommand.VARS.ui8Command = RS485_SET_MOTOR_RUN;
			motorCommand.VARS.ui16Data = 0;
			// Store command
			RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
			break;
		}
		case 2:
		{
			// Arm motor - park
			motorCommand.VARS.ui8Address = motorFRID;
			motorCommand.VARS.ui8Command = RS485_SET_MOTOR_PARK;
			motorCommand.VARS.ui16Data = 0;
			// Store command
			RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
			break;
		}
		case 3:
		{
			// Write motor RPM
			motorCommand.VARS.ui8Address = motorFRID;
			motorCommand.VARS.ui8Command = RS485_SET_MOTOR_RPM;
			motorCommand.VARS.ui16Data = (uint16_t)motorFRSpeed;
			// Store command
			RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
			break;
		}
	}
	return 0;
}

UInt16 RS485_ServoTest(UInt8 servoID)
{
	int bytesToSend = 0;
	RS485COMMAND servoCommand;
	// Store req bytes in tx buffer
	if(servoID == 0)
	{
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SERVO_TORQ_ON;
		servoCommand.VARS.ui16Data = 0;
		bytesToSend = RS485_BufferQueuedCommand(servoCommand);
	}
	else if(servoID == 1)
	{
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SERVO_TORQ_OFF;
		servoCommand.VARS.ui16Data = 0;
		bytesToSend = RS485_BufferQueuedCommand(servoCommand);
	}
	else if(servoID == 2)
	{
		// Store req bytes in tx buffer
		bytesToSend = RS485_Write8(0x01, 0x03, servoFRID);
	}
	else if(servoID == 3)
	{
		// Move servo
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = servoMovePosition;
		bytesToSend = RS485_BufferQueuedCommand(servoCommand);
		//bytesToSend = RS485_ServoWrite16(servoFRID, 0x1e, servoMovePosition);
	}
	else if(servoID == 4)
	{
		// Move servos
		// Servo 1
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = 787;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 2
		servoCommand.VARS.ui8Address = servoFLID;
		servoCommand.VARS.ui16Data = 203;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 3
		servoCommand.VARS.ui8Address = servoRID;
		servoCommand.VARS.ui16Data = 193;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 5)
	{
		// Move servos
		// Servo 1
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = 500;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 2
		servoCommand.VARS.ui8Address = servoFLID;
		servoCommand.VARS.ui16Data = 515;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 3
		servoCommand.VARS.ui8Address = servoRID;
		servoCommand.VARS.ui16Data = 510;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 6)
	{
		// Move servos
		// Servo 1
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = 210;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 2
		servoCommand.VARS.ui8Address = servoFLID;
		servoCommand.VARS.ui16Data = 787;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
		// Servo 3
		servoCommand.VARS.ui8Address = servoRID;
		servoCommand.VARS.ui16Data = 210;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 7)
	{
		// Move servos
		// Servo 1
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_ANGLEMIN_LIM;
		servoCommand.VARS.ui16Data = 183;
		bytesToSend = RS485_BufferQueuedCommand(servoCommand);
	}
	else if(servoID == 8)
	{
		// Move servos
		// Servo 1
		servoCommand.VARS.ui8Address = servoFRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = servoMovePosition;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 9)
	{
		// Move servos
		// Servo 2
		servoCommand.VARS.ui8Address = servoFLID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = servoMovePosition;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 10)
	{
		// Move servos
		// Servo 3
		servoCommand.VARS.ui8Address = servoRID;
		servoCommand.VARS.ui8Command = RS485_SET_SERVO_POSITION;
		servoCommand.VARS.ui16Data = servoMovePosition;
		// Store command
		RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	}
	else if(servoID == 11)
	{
		// Move servos
		RS485CommandDecoder.VARS.ui8Address = servoFRID;
		RS485CommandDecoder.VARS.ui8Command = RS485_POLL_SERVO;
		RS485CommandDecoder.VARS.ui16Data = 0;
		// Store req bytes in tx buffer
		bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
		// Send data with DMA
		RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);

		bytesToSend = 0;

	}


	if(0 != bytesToSend)
	{
		// Send data with DMA
		RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
	}
	return 0;
}

// Queue command
Int16 RS485_TorqueON(UInt8 on)
{
	if(0 != on)
	{

	}
	else
	{

	}
	return 0;
}

Int16 RS485_BufferCommand(UInt8 ID, UInt8 command, UInt16 data)
{
	/*
	RS485COMMAND servoCommand;
	servoCommand.VARS.ui8Address = ID;
	servoCommand.VARS.ui8Command = command;
	servoCommand.VARS.ui16Data = data;
	// Store to command buffer

	//RS485CommandBuffer
*/
	return 0;
}

UInt16 RS485_Write8(UInt8 servoID, UInt8 address, UInt8 data)
{
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;					// ID
	RS485TransmittBuffer[3] = 0x04;						// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[5] = address;					// Reg address
	RS485TransmittBuffer[6] = data;						// Data to write
	// Calculate checksum
	RS485TransmittBuffer[7] = ~(0x07 + servoID + address + data);	// Checksum
	return 8;
}

UInt16 RS485_Write16(UInt8 servoID, UInt8 address, UInt16 data)
{
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;					// ID
	RS485TransmittBuffer[3] = 0x05;						// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[5] = address;					// Reg address
	RS485TransmittBuffer[6] = data & 0xff;				// Data to write low
	RS485TransmittBuffer[7] = (data >> 8)&0xff;			// Data to write high
	// Calculate checksum
	RS485TransmittBuffer[8] = ~(0x08 + servoID + address + RS485TransmittBuffer[6] + RS485TransmittBuffer[7]);	// Checksum
	return 9;
}
UInt16 RS485_Writefloat(UInt8 servoID, UInt8 address, float data)
{

	int i = 0;
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;					// ID
	RS485TransmittBuffer[3] = 0x07;						// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[5] = address;					// Reg address
	RS485ConvertNum.f32[0] = data;
	RS485TransmittBuffer[6] = RS485ConvertNum.ch[0];	// Data to write low
	RS485TransmittBuffer[7] = RS485ConvertNum.ch[1];
	RS485TransmittBuffer[8] = RS485ConvertNum.ch[2];
	RS485TransmittBuffer[9] = RS485ConvertNum.ch[3];	// Data to write high

	// Calculate checksum
	RS485TransmittBuffer[10] = RS485TransmittBuffer[2];
	for(i = 3; i < 10; i++)
	{
		RS485TransmittBuffer[10] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[10] = ~(RS485TransmittBuffer[10]);				// Checksum

	return 11;
}

UInt16 RS485_Read(UInt8 ID, UInt8 readStart, UInt8 readCount)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = ID;					// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_READ;	// Command
	RS485TransmittBuffer[5] = readStart;			// Reg address
	RS485TransmittBuffer[6] = readCount;			// Data bytesd to read

	// Calculate checksum
	RS485TransmittBuffer[7] = RS485TransmittBuffer[2];
	for(i = 3; i < 7; i++)
	{
		RS485TransmittBuffer[7] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[7] = ~(RS485TransmittBuffer[7]);				// Checksum

	// Store read address
	RS485ReadReqStartAddress = RS485TransmittBuffer[5];
	return 8;
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
	RS485TransmittBuffer[7] = ~(0x1f + servoID);	// Checksum
	return 8;
}

UInt16 RS485_ServoReadAll(UInt8 servoID)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = servoID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_READ;	// Command
	RS485TransmittBuffer[5] = 0x18;					// Reg address 24
	RS485TransmittBuffer[6] = 0x1a;//0x49;					// Data to read - 73 regs

	// Calculate checksum
	RS485TransmittBuffer[7] = RS485TransmittBuffer[2];
	for(i = 3; i < 7; i++)
	{
		RS485TransmittBuffer[7] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[7] = ~(RS485TransmittBuffer[7]);				// Checksum

	// Store read address
	RS485ReadReqStartAddress = RS485TransmittBuffer[5];
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

UInt16 RS485_MotorReadAll(UInt8 motorID)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = motorID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_READ;	// Command
	RS485TransmittBuffer[5] = 0x20;					// Reg address 32
	RS485TransmittBuffer[6] = 0x20;					// Data to read - 32 regs

	// Calculate checksum
	RS485TransmittBuffer[7] = RS485TransmittBuffer[2];
	for(i = 3; i < 7; i++)
	{
		RS485TransmittBuffer[7] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[7] = ~(RS485TransmittBuffer[7]);				// Checksum

	// Store read address
	RS485ReadReqStartAddress = RS485TransmittBuffer[5];
	return 8;
}

UInt16 RS485_MotorSetMinSpeed(UInt8 motorID, float32_t minSpeed)
{
	int i = 0;

	RS485ConvertNum.f32[0] = minSpeed;


	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = motorID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x09;					// Reg address 9
	RS485TransmittBuffer[6] = RS485ConvertNum.ch[0];
	RS485TransmittBuffer[7] = RS485ConvertNum.ch[1];
	RS485TransmittBuffer[8] = RS485ConvertNum.ch[2];
	RS485TransmittBuffer[9] = RS485ConvertNum.ch[3];

	// Calculate checksum
	RS485TransmittBuffer[10] = RS485TransmittBuffer[2];
	for(i = 3; i < 10; i++)
	{
		RS485TransmittBuffer[10] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[10] = ~(RS485TransmittBuffer[10]);				// Checksum

	return 0;
}

UInt16 RS485_MotorSetMaxSpeed(UInt8 motorID, float32_t maxSpeed)
{
	int i = 0;

	RS485ConvertNum.f32[0] = maxSpeed;


	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = motorID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x0d;					// Reg address 13
	RS485TransmittBuffer[6] = RS485ConvertNum.ch[0];
	RS485TransmittBuffer[7] = RS485ConvertNum.ch[1];
	RS485TransmittBuffer[8] = RS485ConvertNum.ch[2];
	RS485TransmittBuffer[9] = RS485ConvertNum.ch[3];

	// Calculate checksum
	RS485TransmittBuffer[10] = RS485TransmittBuffer[2];
	for(i = 3; i < 10; i++)
	{
		RS485TransmittBuffer[10] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[10] = ~(RS485TransmittBuffer[10]);				// Checksum

	return 0;
}

UInt16 RS485_MotorEnablePWMs(UInt8 motorID, UInt16 enable)
{
	int i = 0;
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = motorID;				// ID
	RS485TransmittBuffer[3] = 0x04;					// Length
	RS485TransmittBuffer[4] = RS485_COMMAND_WRITE;	// Command
	RS485TransmittBuffer[5] = 0x11;					// Reg address 17
	RS485TransmittBuffer[6] = (UInt8)enable;		// Data

	// Calculate checksum
	RS485TransmittBuffer[7] = RS485TransmittBuffer[2];
	for(i = 3; i < 7; i++)
	{
		RS485TransmittBuffer[7] += RS485TransmittBuffer[i];
	}
	RS485TransmittBuffer[7] = ~(RS485TransmittBuffer[7]);				// Checksum

	// Store read address
	RS485ReadReqStartAddress = 0;
	return 8;
}



UInt16 RS485_BufferQueuedCommand(RS485COMMAND command)
{
	UInt16 bytes = 0;
	switch(command.VARS.ui8Command)
	{
		case RS485_POLL_SERVO:
		{
			bytes = RS485_ServoReadAll(command.VARS.ui8Address);
			break;
		}
		case RS485_POLL_MOTOR:
		{
			bytes = RS485_MotorReadAll(command.VARS.ui8Address);
			break;
		}
		case RS485_SERVO_TORQ_ON:
		{
			bytes = RS485_ServoTorqueON(command.VARS.ui8Address);
			break;
		}
		case RS485_SERVO_TORQ_OFF:
		{
			bytes = RS485_ServoTorqueOFF(command.VARS.ui8Address);
			break;
		}
		case RS485_SET_SERVO_POSITION:
		{
			bytes = RS485_ServoSetPosition(command.VARS.ui8Address, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_MOTOR_SPEED:
		{
			bytes = RS485_ServoSetSpeed(command.VARS.ui8Address, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_MOTOR_MIN_SPEED:
		{
			bytes = RS485_MotorSetMinSpeed(command.VARS.ui8Address, (float32_t)command.VARS.ui16Data);
			break;
		}
		case RS485_SET_MOTOR_MAX_SPEED:
		{
			bytes = RS485_MotorSetMaxSpeed(command.VARS.ui8Address, (float32_t)command.VARS.ui16Data);
			break;
		}
		case RS485_ENABLE_MOTOR_PWMIN:
		{
			bytes = RS485_MotorEnablePWMs(command.VARS.ui8Address, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_SERVO_ANGLEMIN_LIM:
		{
			bytes = RS485_Write16(command.VARS.ui8Address, 0x06, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_SERVO_ANGLEMAX_LIM:
		{
			bytes = RS485_Write16(command.VARS.ui8Address, 0x08, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_SERVO_SPEED:
		{
			bytes = RS485_Write16(command.VARS.ui8Address, 0x20, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_MOTOR_POSITION:
		{
			bytes = RS485_Write16(command.VARS.ui8Address, MOTORREG_PARKPOSITION, command.VARS.ui16Data);
			break;
		}
		case RS485_SET_MOTOR_PARK:
		{
			bytes = RS485_Write8(command.VARS.ui8Address, MOTORREG_PARK, 1);
			break;
		}
		case RS485_SET_MOTOR_RUN:
		{
			bytes = RS485_Write8(command.VARS.ui8Address, MOTORREG_ARMED, 1);
			break;
		}
		case RS485_SET_MOTOR_RPM:
		{
			bytes = RS485_Writefloat(command.VARS.ui8Address, MOTORREG_SETRPM, (float)command.VARS.ui16Data);
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


UInt16 RS485_States_Master()
{
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
				if(0 != readRS485Data)
				{
					// Go through RS485 slaves and request data
					switch(RS485MasterPollState)
					{
						case RS485_POLL_STATE_SERVO_FR:
						{
							RS485CommandDecoder.VARS.ui8Address = servoFRID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_SERVO;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_SERVO_FL;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
							break;
						}
						case RS485_POLL_STATE_SERVO_FL:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = servoFLID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_SERVO;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_SERVO_R;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
							break;
						}
						case RS485_POLL_STATE_SERVO_R:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = servoRID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_SERVO;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							//RS485MasterPollState = RS485_POLL_STATE_MOTOR_FR;
							readRS485Data = 0;
							RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;

							break;
						}
						case RS485_POLL_STATE_MOTOR_FR:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = motorFRID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_MOTOR;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_MOTOR_FL;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
							break;
						}
						case RS485_POLL_STATE_MOTOR_FL:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = motorFLID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_MOTOR;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_MOTOR_R;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
							break;
						}
						case RS485_POLL_STATE_MOTOR_R:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = motorRID;
							RS485CommandDecoder.VARS.ui8Command = RS485_POLL_MOTOR;
							RS485CommandDecoder.VARS.ui16Data = 0;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
							readRS485Data = 0;
							break;
						}
						default:
						{
							RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
							break;
						}
					}
				}

			}
			else
			{
				// Get command from buffer
				RS485CommandDecoder.ui32Packed = RB32_pop(&RS485CommandBuffer);
				// Store req bytes in tx buffer
				bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
				// Send data with DMA
				RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
				RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
			}
			// Go to waiting for response state
			RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
			// Reset master waiting timer
			RS485TimerCount = 0;
			// Set no response received
			RS485ResponseReceived = 0;
			break;
		}

		case RS485_M_STATE_WAITING_RESPONSE:
		{
			RS485TimerCount++;
			if(RS485TimerCount > RS485TimerTimeout)
			{
				// Timeout, no response from slave device.
				// Go back to poll state
				RS485MasterState = RS485_M_STATE_DELAY;
			}
			if(0 != RS485ResponseReceived)
			{
				// Go back to poll state
				RS485MasterState = RS485_M_STATE_DELAY;
			}
			break;
		}

		case RS485_M_STATE_DELAY:
		{
			// Reset master waiting timer
			RS485TimerCount = 0;
			RS485PollInterval--;
			if(0 > RS485PollInterval)
			{
				RS485MasterState = RS485_M_STATE_POLL;
				RS485PollInterval = 10;
			}
			break;
		}

		default:
		{
			RS485MasterState = RS485_M_STATE_POLL;
			break;
		}
	}
	return 0;
}

UInt16 RS485_ReceiveMessage(UInt8 data)
{
	// Reset RX timer
	i16RS485TimeOut = 100;
	// Reset master waiting timer
	RS485TimerCount = 0;
	switch(RS485ReceiveState)
	{
		case RS485_M_IDLE:
		{
			// Only if we are waiting for data
			if(RS485_M_STATE_WAITING_RESPONSE == RS485MasterState)
			{
				// Do we have 0xff?
				if(0xff == data)
				{
					RS485ReceiveState = RS485_M_WAIT_FOR_SIGNAL;
				}
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
			else
			{
				RS485ReceiveState = RS485_M_IDLE;
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
			RS485RXBytesLeft = data - 1;
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
				// Mark response received - store ID
				RS485ResponseReceived = RS485RXBuffer[0];
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
	int destIndex = 0;
	int sourceIndex = 3;
	int bytes = 0;
	// Check ID
	switch(RS485RXBuffer[0])
	{
		case SERVO_FR_ID:
		{
			// Store error status
			RS485Servo_FR.errStatus = RS485RXBuffer[2];
			// Store data bytes
			destIndex = RS485ReadReqStartAddress;
			for(bytes = (RS485RXBuffer[1] - 2); bytes != 0; bytes--)
			{
				RS485Servo_FR.ui8REGSData[destIndex] = RS485RXBuffer[sourceIndex];
				destIndex++;
				sourceIndex++;
			}
			break;
		}
		case SERVO_FL_ID:
		{
			// Store error status
			RS485Servo_FL.errStatus = RS485RXBuffer[2];
			// Store data bytes
			destIndex = RS485ReadReqStartAddress;
			for(bytes = (RS485RXBuffer[1] - 2); bytes != 0; bytes--)
			{
				RS485Servo_FL.ui8REGSData[destIndex] = RS485RXBuffer[sourceIndex];
				destIndex++;
				sourceIndex++;
			}
			break;
		}
		case SERVO_R_ID:
		{
			// Store error status
			RS485Servo_R.errStatus = RS485RXBuffer[2];
			// Store data bytes
			destIndex = RS485ReadReqStartAddress;
			for(bytes = (RS485RXBuffer[1] - 2); bytes != 0; bytes--)
			{
				RS485Servo_R.ui8REGSData[destIndex] = RS485RXBuffer[sourceIndex];
				destIndex++;
				sourceIndex++;
			}
			break;
		}
		case MOTOR_FR_ID:
		{
			// Store error status
			RS485Motor_FR.errStatus = RS485RXBuffer[2];
			// Store data bytes
			destIndex = RS485ReadReqStartAddress;
			for(bytes = (RS485RXBuffer[1] - 2); bytes != 0; bytes--)
			{
				RS485Motor_FR.ui8Data[destIndex] = RS485RXBuffer[sourceIndex];
				destIndex++;
				sourceIndex++;
			}
			break;
		}
	}
	return 0;
}

// Setup motors
Int16 RS485_SetupServos()
{
	RS485COMMAND servoCommand;

	// Set servo min angle ***************************
	// Servo 1
	servoCommand.VARS.ui8Address = servoFRID;
	servoCommand.VARS.ui8Command = RS485_SET_SERVO_ANGLEMIN_LIM;
	servoCommand.VARS.ui16Data = 183;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 2
	servoCommand.VARS.ui8Address = servoFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 3
	servoCommand.VARS.ui8Address = servoRID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);

	// Set servo max angle ***************************
	// Servo 1
	servoCommand.VARS.ui8Address = servoFRID;
	servoCommand.VARS.ui8Command = RS485_SET_SERVO_ANGLEMAX_LIM;
	servoCommand.VARS.ui16Data = 797;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 2
	servoCommand.VARS.ui8Address = servoFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 3
	servoCommand.VARS.ui8Address = servoRID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);

	// Set servo speed ***************************
	// Servo 1
	servoCommand.VARS.ui8Address = servoFRID;
	servoCommand.VARS.ui8Command = RS485_SET_SERVO_SPEED;
	servoCommand.VARS.ui16Data = 100;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 2
	servoCommand.VARS.ui8Address = servoFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 3
	servoCommand.VARS.ui8Address = servoRID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);

	// Set torque ON ***************************
	// Servo 1
	servoCommand.VARS.ui8Address = servoFRID;
	servoCommand.VARS.ui8Command = RS485_SERVO_TORQ_ON;
	servoCommand.VARS.ui16Data = 0;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 2
	servoCommand.VARS.ui8Address = servoFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
	// Servo 3
	servoCommand.VARS.ui8Address = servoRID;
	// Store command
	RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);


	return 0;
}


// Setup motors
Int16 RS485_SetupMotors()
{
	RS485COMMAND motorCommand;

	// Set motor min RPM ***************************
	// Motor 1
	motorCommand.VARS.ui8Address = motorFRID;
	motorCommand.VARS.ui8Command = RS485_SET_MOTOR_MIN_SPEED;
	motorCommand.VARS.ui16Data = 500;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 2
	motorCommand.VARS.ui8Address = motorFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 3
	motorCommand.VARS.ui8Address = motorRID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);

	// Set motor max RPM ***************************
	// Motor 1
	motorCommand.VARS.ui8Address = motorFRID;
	motorCommand.VARS.ui8Command = RS485_SET_MOTOR_MAX_SPEED;
	motorCommand.VARS.ui16Data = 10000;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 2
	motorCommand.VARS.ui8Address = motorFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 3
	motorCommand.VARS.ui8Address = motorRID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);

	// Enable PWM inputs ***************************
	// Motor 1
	motorCommand.VARS.ui8Address = motorFRID;
	motorCommand.VARS.ui8Command = RS485_ENABLE_MOTOR_PWMIN;
	motorCommand.VARS.ui16Data = 1;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 2
	motorCommand.VARS.ui8Address = motorFLID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);
	// Motor 3
	motorCommand.VARS.ui8Address = motorRID;
	// Store command
	RB32_push(&RS485CommandBuffer, motorCommand.ui32Packed);



	return 0;
}
