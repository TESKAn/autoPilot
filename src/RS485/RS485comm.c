/*
 * RS485comm.c
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */


#include "allinclude.h"

// Disable timeouts
//#define RS485_NOTIMEOUT

// Default defines
#define SERVO_FR_ID		0x13
#define SERVO_FL_ID		0x12
#define SERVO_R_ID		0x11
#define MOTOR_FR_ID		0x23
#define MOTOR_FL_ID		0x22
#define MOTOR_R_ID		0x21

// Motor read regs
#define MOTOR_START_READ_REG	10
#define MOTOR_COUNT_READ_REG	45

#define SERVO_START_READ_REG	24
#define SERVO_COUNT_READ_REG	25


UInt8 servoFRID = SERVO_FR_ID;
UInt8 servoFLID = SERVO_FL_ID;
UInt8 servoRID = SERVO_R_ID;

UInt8 motorFRID = MOTOR_FR_ID;
UInt8 motorFLID = MOTOR_FL_ID;
UInt8 motorRID = MOTOR_R_ID;

Int8 RS485PollInterval = 10;
Int8 RS485PollDelay = 120;

RS485SERVO RS485Servo_FL;
RS485SERVO RS485Servo_FR;
RS485SERVO RS485Servo_R;

RS485MOTOR RS485Motor_FL;
RS485MOTOR RS485Motor_FR;
RS485MOTOR RS485Motor_R;

// Struct for receiving data
RS485RXDATA RS485DataStruct;
RS485RXDATA* RS485Data;

// Store info of unit that we are talking to
RS485WAITINGUNIT RS485CurrentUnit;
RS485COMMAND RS485CurrentCommand;

UInt8 RS485TransmittBuffer[64];

UInt8 RS485BytesToReceive = 0;

UInt8 RS485MasterState = RS485_M_STATE_IDLE;
UInt8 RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
UInt8 RS485MasterPoll = 0;

// Poll retries
UInt8 RS485PollRetries = 0;

// 500 ms timeout
UInt16 RS485TimerCount = 0;
UInt16 RS485TimerTimeout = 1000;
UInt8 RS485ResponseReceived = 0;

UInt8 RS485ReceiveState = 0;
UInt8 RS485RXBufIndex = 0;
UInt8 RS485RXBytesLeft = 0;
UInt8 RS485Checksum = 0;
UInt8 RS485ReadReqStartAddress = 0;

// Variables
UInt32 RS485_CommandBuffer[32];

RING_BUFFER32 RS485CommandBuffer;

RS485COMMAND RS485CommandDecoder;

CONVERTNUM RS485ConvertNum;

Int16 i16RS485Timing = 0;

Int16 i16RS485TimeOut = 100;


Int16 RS485_Timing()
{
	// Check RX process
#ifndef RS485_NOTIMEOUT

	if(RS485_RX_IDLE != RS485Data->ui8RXState)
	{
		RS485Data->ui16RXTimeoutCounter++;
		if(RS485Data->ui16RXTimeoutCounter > RS485Data->ui16RXCommTimeout)
		{
			// Try one more time
			/*
			uint16_t reg = USART1->SR;
			uint16_t reg1 = USART1->CR1;
			uint16_t reg2 = USART1->CR2;
			uint16_t reg3 = USART1->CR3;
			uint16_t reg4 = USART1->BRR;
			uint16_t reg5 = USART1->DR;
			uint8_t bit5 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
			*/
			uint16_t reg1 = USART1->DR;
			RS485_ReceiveMessage((uint8_t)reg1);
			if(1 != RS485ResponseReceived)
			{
				RS485Data->ui16RXTimeoutCounter = 0;
				RS485Data->ui8RXState = RS485_RX_IDLE;
				RS485Data->ui8RXCounter = 0;
				RB_flush(&RB_USART1);
			}

		}
	}
#endif
	return 0;
}

// Queue write to 16 bit slave reg
Int16 RS485_WriteSlaveReg16(UInt8 ID, UInt8 reg, UInt16 data)
{
	RS485COMMAND Command;
	// Servo
	Command.VARS.ui8Address = ID;
	Command.VARS.ui8Command = reg & 0x7f;
	Command.VARS.ui16Data = data;
	// Store command
	RB32_push(&RS485CommandBuffer, Command.ui32Packed);
	return 0;
}

// Queue write to 8 bit slave reg
Int16 RS485_WriteSlaveReg8(UInt8 ID, UInt8 reg, UInt8 data)
{
	RS485COMMAND Command;
	// Servo
	Command.VARS.ui8Address = ID;
	Command.VARS.ui8Command = reg | 0x80;
	Command.VARS.ui16Data = data;
	// Store command
	RB32_push(&RS485CommandBuffer, Command.ui32Packed);
	return 0;
}

Int16 RS485_ReadSlave(UInt8 ID, UInt8 startReg, UInt8 count)
{
	RS485COMMAND Command;
	// Servo
	Command.VARS.ui8Address = ID | 0x80;
	Command.VARS.ui8Command = startReg;
	Command.VARS.ui16Data = (UInt16)count;
	// Store command
	RB32_push(&RS485CommandBuffer, Command.ui32Packed);
	return 0;
}

// Master functions
Int16 RS485_MasterInitData(void)
{
	// Set slaves
	RS485Servo_FR.REGS.ui8ID = servoFRID;
	RS485Servo_FL.REGS.ui8ID = servoFLID;
	RS485Servo_R.REGS.ui8ID = servoRID;

	RS485Motor_FR.REGS.ui8ID = motorFRID;
	RS485Motor_FL.REGS.ui8ID = motorFLID;
	RS485Motor_R.REGS.ui8ID = motorRID;

	RS485Data = &RS485DataStruct;

	// Set timeout
	RS485Data->ui16RXCommTimeout = 500;

	// Init command buffer
	RB32_Init(&RS485CommandBuffer, RS485_CommandBuffer, 16);

	RS485Motor_R.REGS.i16ParkPosition = 2048;
	RS485Motor_R.REGS.i16ZeroSpeedPWM = 50;

	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	return 0;
}

// Queue a command
Int16 RS485_QueueCommand(RS485COMMAND cmdToExec)
{
	// Store command
	return RB32_push(&RS485CommandBuffer, cmdToExec.ui32Packed);
}

Int16 RS485_MasterState(int state)
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
	return 0;
}

Int16 RS485_ServoTest(UInt8 func)
{/*
	RS485COMMAND servoCommand;

	switch(func)
	{
		case 0:
		{
			servoCommand.VARS.ui8Address = servoFRID;
			//servoCommand.VARS.ui8Command = RS485_SERVO_TORQ_ON;
			servoCommand.VARS.ui16Data = 0;
			// Store command
			RB32_push(&RS485CommandBuffer, servoCommand.ui32Packed);
			break;
		}
	}*/
	return 0;
}


UInt16 RS485_Write8(UInt8 ID, UInt16 address, UInt8 data)
{
	union
	{
		UInt16 data;
		UInt8 bytes[2];
	}ui16Val;
	ui16Val.data = address;
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = 0xfd;
	RS485TransmittBuffer[3] = 0x00;
	RS485TransmittBuffer[4] = ID;						// ID
	RS485TransmittBuffer[5] = 0x06;						// Length
	RS485TransmittBuffer[6] = 0x00;						// Length
	RS485TransmittBuffer[7] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[8] = ui16Val.bytes[0];			// Reg address
	RS485TransmittBuffer[9] = ui16Val.bytes[1];			// Reg address
	RS485TransmittBuffer[10] = data;					// Data to write
	// Calculate checksum
	ui16Val.data = update_crc(0, RS485TransmittBuffer, 11);
	RS485TransmittBuffer[11] = ui16Val.bytes[0];
	RS485TransmittBuffer[12] = ui16Val.bytes[1];
	// Store what was sent
	RS485CurrentUnit.ui8ID = ID;
	RS485CurrentUnit.ui8Instruction = RS485_COMMAND_WRITE;
	RS485CurrentUnit.ui16RegAddress = address;
	RS485CurrentUnit.ui16ByteCount = 6;
	return 13;
}

UInt16 RS485_Write16(UInt8 ID, UInt16 address, UInt16 data)
{
	union
	{
		UInt16 data;
		UInt8 bytes[2];
	}ui16Val;
	ui16Val.data = address;
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = 0xfd;
	RS485TransmittBuffer[3] = 0x00;
	RS485TransmittBuffer[4] = ID;						// ID
	RS485TransmittBuffer[5] = 0x07;						// Length
	RS485TransmittBuffer[6] = 0x00;						// Length
	RS485TransmittBuffer[7] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[8] = ui16Val.bytes[0];			// Reg address
	RS485TransmittBuffer[9] = ui16Val.bytes[1];			// Reg address
	ui16Val.data = data;
	RS485TransmittBuffer[10] = ui16Val.bytes[0];			// Data to write
	RS485TransmittBuffer[11] = ui16Val.bytes[1];			// Data to write
	// Calculate checksum
	ui16Val.data = update_crc(0, RS485TransmittBuffer, 12);
	RS485TransmittBuffer[12] = ui16Val.bytes[0];
	RS485TransmittBuffer[13] = ui16Val.bytes[1];
	// Store what was sent
	RS485CurrentUnit.ui8ID = ID;
	RS485CurrentUnit.ui8Instruction = RS485_COMMAND_WRITE;
	RS485CurrentUnit.ui16RegAddress = address;
	RS485CurrentUnit.ui16ByteCount = 7;
	return 14;
}
UInt16 RS485_Writefloat(UInt8 ID, UInt16 address, float data)
{
	union
	{
		UInt16 data;
		UInt8 bytes[2];
	}ui16Val;
	ui16Val.data = address;
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = 0xfd;
	RS485TransmittBuffer[3] = 0x00;
	RS485TransmittBuffer[4] = ID;					// ID
	RS485TransmittBuffer[5] = 0x09;						// Length
	RS485TransmittBuffer[6] = 0x00;						// Length
	RS485TransmittBuffer[7] = RS485_COMMAND_WRITE;		// Command
	RS485TransmittBuffer[8] = ui16Val.bytes[0];			// Reg address
	RS485TransmittBuffer[9] = ui16Val.bytes[1];			// Reg address
	RS485ConvertNum.f32[0] = data;
	RS485TransmittBuffer[10] = RS485ConvertNum.ch[0];	// Data to write low
	RS485TransmittBuffer[11] = RS485ConvertNum.ch[1];
	RS485TransmittBuffer[12] = RS485ConvertNum.ch[2];
	RS485TransmittBuffer[13] = RS485ConvertNum.ch[3];	// Data to write high
	// Calculate checksum
	ui16Val.data = update_crc(0, RS485TransmittBuffer, 14);
	RS485TransmittBuffer[14] = ui16Val.bytes[0];
	RS485TransmittBuffer[15] = ui16Val.bytes[1];
	// Store what was sent
	RS485CurrentUnit.ui8ID = ID;
	RS485CurrentUnit.ui8Instruction = RS485_COMMAND_WRITE;
	RS485CurrentUnit.ui16RegAddress = address;
	RS485CurrentUnit.ui16ByteCount = 9;
	return 16;
}

UInt16 RS485_Read(UInt8 ID, UInt16 address, UInt16 count)
{
	union
	{
		UInt16 data;
		UInt8 bytes[2];
	}ui16Val;
	ui16Val.data = address;
	// Build message
	RS485TransmittBuffer[0] = 0xff;
	RS485TransmittBuffer[1] = 0xff;
	RS485TransmittBuffer[2] = 0xfd;
	RS485TransmittBuffer[3] = 0x00;
	RS485TransmittBuffer[4] = ID;						// ID
	RS485TransmittBuffer[5] = 0x07;						// Length
	RS485TransmittBuffer[6] = 0x00;						// Length
	RS485TransmittBuffer[7] = RS485_COMMAND_READ;		// Command
	RS485TransmittBuffer[8] = ui16Val.bytes[0];			// Reg address
	RS485TransmittBuffer[9] = ui16Val.bytes[1];			// Reg address
	ui16Val.data = count;
	RS485TransmittBuffer[10] = ui16Val.bytes[0];			// Data to write
	RS485TransmittBuffer[11] = ui16Val.bytes[1];			// Data to write
	// Calculate checksum
	ui16Val.data = update_crc(0, RS485TransmittBuffer, 12);
	RS485TransmittBuffer[12] = ui16Val.bytes[0];
	RS485TransmittBuffer[13] = ui16Val.bytes[1];
	// Store what was sent
	RS485CurrentUnit.ui8ID = ID;
	RS485CurrentUnit.ui8Instruction = RS485_COMMAND_READ;
	RS485CurrentUnit.ui16RegAddress = address;
	RS485CurrentUnit.ui16ByteCount = count;
	return 14;
}

UInt16 RS485_BufferQueuedCommand(RS485COMMAND command)
{
	UInt16 bytes = 0;
	// Store additional data in ui8Command bit 7
	if(0 != (command.VARS.ui8Address & 0x80))
	{
		// Read?
		command.VARS.ui8Address = command.VARS.ui8Address & 0x7f;
		bytes = RS485_Read(command.VARS.ui8Address, (UInt16)command.VARS.ui8Command, command.VARS.ui16Data);
	}
	else
	{
		// Else write data
		if(0 != (command.VARS.ui8Command & 0x80))
		{
			// 8 bit var
			command.VARS.ui8Command = command.VARS.ui8Command & 0x7f;
			bytes = RS485_Write8(command.VARS.ui8Address, command.VARS.ui8Command, (UInt8)command.VARS.ui16Data);
		}
		else
		{
			// 16 bit var
			command.VARS.ui8Command = command.VARS.ui8Command & 0x7f;
			bytes = RS485_Write16(command.VARS.ui8Address, command.VARS.ui8Command, command.VARS.ui16Data);
		}
	}
	return bytes;
}


Int16 RS485_MasterWriteByte(uint8_t *data, int length)
{
	// Transmit data
	// Flush RX buffer
	RB_flush(&RB_USART1);
	// Enable transmit
	RS485_TXEN;
	// Send
	transferDMA_USART1(data, length);
	return 0;
}


void RS485_States_Master()
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
#ifndef RS485_DISABLE_POLL
				RS485PollDelay--;
				if(0 == RS485PollDelay)
				{
					RS485PollDelay = 120;
					// Go through RS485 slaves and request data
					switch(RS485MasterPollState)
					{
						case RS485_POLL_STATE_SERVO_FR:
						{
							RS485CommandDecoder.VARS.ui8Address = servoFRID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = SERVO_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = SERVO_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
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
							RS485CommandDecoder.VARS.ui8Address = servoFLID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = SERVO_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = SERVO_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
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
							RS485CommandDecoder.VARS.ui8Address = servoRID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = SERVO_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = SERVO_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
							// Store req bytes in tx buffer
							bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
							// Send data with DMA
							RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
							// Set next
							RS485MasterPollState = RS485_POLL_STATE_MOTOR_FR;
							//readRS485Data = 0;
							//RS485MasterPollState = RS485_POLL_STATE_SERVO_FR;
							RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;

							break;
						}
						case RS485_POLL_STATE_MOTOR_FR:
						{
							// Store req bytes in tx buffer
							RS485CommandDecoder.VARS.ui8Address = motorFRID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = MOTOR_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = MOTOR_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
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
							RS485CommandDecoder.VARS.ui8Address = motorFLID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = MOTOR_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = MOTOR_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
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
							RS485CommandDecoder.VARS.ui8Address = motorRID | 0x80;
							RS485CommandDecoder.VARS.ui8Command = MOTOR_START_READ_REG;
							RS485CommandDecoder.VARS.ui16Data = MOTOR_COUNT_READ_REG;
							// Store polling ID
							RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
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
							readRS485Data = 0;
							break;
						}
					}
				}
#endif
			}
			else
			{
				// Get command from buffer
				RS485CommandDecoder.ui32Packed = RB32_pop(&RS485CommandBuffer);
				// Store last command sent
				RS485CurrentCommand.VARS.ui8Address = RS485CommandDecoder.VARS.ui8Address;
				RS485CurrentCommand.VARS.ui8Command = RS485CommandDecoder.VARS.ui8Command;
				RS485CurrentCommand.VARS.ui16Data = RS485CommandDecoder.VARS.ui16Data;
				// Store req bytes in tx buffer
				bytesToSend = RS485_BufferQueuedCommand(RS485CommandDecoder);
				// Send data with DMA
				RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
				RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
				// retries to send
				RS485PollRetries = 5;
			}
			// Did we send data?
			if(0 < bytesToSend)
			{
				// Go to waiting for response state
				RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
				// Reset master waiting timer
				RS485TimerCount = 0;
				// Set no response received
				RS485ResponseReceived = 0;
			}
			else
			{

			}

			break;
		}

		case RS485_M_STATE_WAITING_RESPONSE:
		{
			RS485TimerCount++;
			if(RS485TimerCount > RS485TimerTimeout)
			{
				UART_QueueMessageui16(VAR_RS485_NO_RESPONSE_ID, (UInt16)(RS485CurrentCommand.VARS.ui8Address & 0x7f));

				RS485TimerCount = 0;
				ui32TestVar++;
				// Timeout, no response from slave device.
				if(0 < RS485PollRetries)
				{
					RS485PollRetries--;
					// Resend message
					// Store req bytes in tx buffer
					bytesToSend = RS485_BufferQueuedCommand(RS485CurrentCommand);
					// Send data with DMA
					RS485_MasterWriteByte(RS485TransmittBuffer, bytesToSend);
					RS485MasterState = RS485_M_STATE_WAITING_RESPONSE;
				}
				else
				{
					// Go back to poll state
					RS485MasterState = RS485_M_STATE_DELAY;
				}
			}
			else if(0 != RS485ResponseReceived)
			{
				// Slave responded
				RS485PollRetries = 0;
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
				RS485PollInterval = 20;
			}
			break;
		}

		default:
		{
			RS485MasterState = RS485_M_STATE_POLL;
			break;
		}
	}
}

void RS485_ReceiveMessage(UInt8 data)
{
	UInt16 ui16Temp = 0;
	RS485Data->ui16RXTimeoutCounter = 0;
	// Store
	RS485Data->RXDATA.ui8Parameters[RS485Data->RXDATA.ui8RS485RXIndex] = data;
	RS485Data->RXDATA.ui8RS485RXIndex++;
	if(127 <= RS485Data->RXDATA.ui8RS485RXIndex)
	{
		RS485Data->RXDATA.ui8RS485RXIndex = 0;
	}
	switch(RS485Data->ui8RXState)
	{
		case RS485_RX_IDLE:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for signal
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_SIGNAL;
				RS485Data->RXDATA.HEADER.ui8Bytes[0] = data;
				RS485Data->ui8RXCounter = 1;
				RS485Data->RXDATA.ui8Parameters[0] = data;
				RS485Data->RXDATA.ui8RS485RXIndex = 1;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_SIGNAL:
		{
			if(3 > RS485Data->ui8RXCounter)
			{
				RS485Data->RXDATA.HEADER.ui8Bytes[RS485Data->ui8RXCounter] = data;
				RS485Data->ui8RXCounter ++;
			}
			else
			{
				// Check header
				if(0x00fdffff == RS485Data->RXDATA.HEADER.ui32Header)
				{
					RS485Data->ui8RXState = RS485_RX_WAIT_FOR_ID;
					RS485Data->ui8RXCounter = 0;
				}
				else
				{
					RS485Data->ui8RXState = RS485_RX_IDLE;
					RS485Data->ui8RXCounter = 0;
				}
			}
			break;
		}
		case RS485_RX_WAIT_FOR_ID:
		{
			// Data == ID?
			if(data == RS485CurrentUnit.ui8ID)
			{
				// ID match, wait for data length
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_LENGTH;
				RS485Data->ui8RXCounter = 0;
				// Store ID
				RS485Data->RXDATA.ui8ID = data;
			}
			else
			{
				// Id not matched, go to idle
				RS485Data->ui8RXState = RS485_RX_IDLE;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_LENGTH:
		{
			// Store byte
			RS485Data->RXDATA.LENGTH.ui8Bytes[RS485Data->ui8RXCounter] = data;
			RS485Data->ui8RXCounter++;
			if(2 == RS485Data->ui8RXCounter)
			{
				// Store number of bytes in parameters, is length + 5 bytes for total packet length - crc
				RS485Data->RXDATA.ui8ParamByteCount = RS485Data->RXDATA.LENGTH.ui16PacketLength + 5;
				// Go to wait for instruction
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_INSTR_ERR;
				RS485Data->ui8RXCounter = 0;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_INSTR_ERR:
		{
			// Store instruction
			RS485Data->RXDATA.ui8Instruction = data;
			if(3 == RS485Data->RXDATA.LENGTH.ui16PacketLength)
			{
				// No parameters, only instruction + CRC
				// Wait for checksum
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
				RS485Data->ui8RXCounter = 0;
			}
			else
			{
				// Waiting for parameters
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_PARAMETERS;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_PARAMETERS:
		{
			// Have all parameters?
			if(RS485Data->RXDATA.ui8ParamByteCount == RS485Data->RXDATA.ui8RS485RXIndex)
			{
				// Yes, go to receive checksum
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
				RS485Data->ui8RXCounter = 0;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_CHECKSUM:
		{
			RS485Data->RXDATA.CRCDATA.ui8Bytes[RS485Data->ui8RXCounter] = data;
			RS485Data->ui8RXCounter++;
			if(2 == RS485Data->ui8RXCounter)
			{

				// Calculate CRC
				ui16Temp = update_crc(0, RS485Data->RXDATA.ui8Parameters, 5 + RS485Data->RXDATA.LENGTH.ui16PacketLength);
				if(ui16Temp == RS485Data->RXDATA.CRCDATA.ui16CRC)
				{
					// Message OK, decode
					RS485_DecodeMessage();
				}
				RS485Data->ui8RXState = RS485_RX_IDLE;
				RS485Data->ui8RXCounter = 0;
			}

			break;
		}
		default:
		{
			RS485Data->ui8RXState = RS485_RX_IDLE;
		}
	}
}

Int16 RS485_DecodeMessage()
{
	int destIndex = 0;
	int bytes = 0;
	UInt8* ui8DestRegAddress;
	UInt8* ui8FreshData;
	// Mark we have response
	RS485ResponseReceived = 1;
	// Check where to put data
	switch(RS485Data->RXDATA.ui8ID)
	{
		case SERVO_FR_ID:
		{
			// Error status
			RS485Servo_FR.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Servo_FR.ui8REGSData;
			ui8FreshData = &RS485Servo_FR.ui8FreshData;
			break;
		}
		case SERVO_FL_ID:
		{
			// Error status
			RS485Servo_FL.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Servo_FL.ui8REGSData;
			ui8FreshData = &RS485Servo_FL.ui8FreshData;
			break;
		}
		case SERVO_R_ID:
		{
			// Error status
			RS485Servo_R.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Servo_R.ui8REGSData;
			ui8FreshData = &RS485Servo_R.ui8FreshData;
			break;
		}
		case MOTOR_FR_ID:
		{
			// Error status
			RS485Motor_FR.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Motor_FR.ui8REGSData;
			ui8FreshData = &RS485Motor_FR.ui8FreshData;
			break;
		}
		case MOTOR_FL_ID:
		{
			// Error status
			RS485Motor_FL.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Motor_FL.ui8REGSData;
			ui8FreshData = &RS485Motor_FL.ui8FreshData;
			break;
		}
		case MOTOR_R_ID:
		{
			// Error status
			RS485Motor_R.errStatus = RS485Data->RXDATA.ui8Parameters[8];
			ui8DestRegAddress = RS485CurrentUnit.ui16RegAddress + RS485Motor_R.ui8REGSData;
			ui8FreshData = &RS485Motor_R.ui8FreshData;
			break;
		}
	}

	if(RS485_COMMAND_READ == RS485CurrentUnit.ui8Instruction)
	{
		// Write data to registers
		destIndex = 0;
		// Fix pointer
		ui8DestRegAddress --;
		for(bytes = 0; bytes < RS485CurrentUnit.ui16ByteCount; bytes++)
		{
			ui8DestRegAddress[destIndex] = RS485Data->RXDATA.ui8Parameters[bytes + 9];
			destIndex++;
		}
		// Mark fresh data
		if(3 < RS485Data->RXDATA.LENGTH.ui16PacketLength)
		{
			*ui8FreshData = 1;
		}
	}
	return 0;
}

// Setup motors
Int16 RS485_SetupServos()
{
	return 0;
}


// Setup motors
Int16 RS485_SetupMotors()
{
	return 0;
}

UInt16 update_crc(UInt16 crc_accum, UInt8 *data_blk_ptr, UInt16 data_blk_size)
{
	UInt16 i, j;
	UInt16 crc_table[256] = {

        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

    };
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((UInt16)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

