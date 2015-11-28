/*
 * RS485comm.c
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */


#include "allinclude.h"


RS485SERVO RS485Data;

RS485SERVOMASTER RS485Master;

UInt16 ui16RS485Timer = 5000;
UInt8 ui8RS485TestSequence = 0;
UInt8 ui8Temp0;


// Master functions
UInt16 RS485_MasterInitData(void)
{
	Int16 i16Temp = 0;
	// Set slaves
	// ID's
	RS485Master.RS485Slaves[0].REGS.ui8ID = 1;
	// Common regs
	for(i16Temp = 0; i16Temp < RS485_NUMSLAVES; i16Temp++)
	{

	}
	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	// Set states
	RS485Master.ui8RcvState = RS485_M_IDLE;
	RS485Master.ui8TxState = RS485_M_TX_IDLE;
	return 0;
}

UInt16 RS485_MasterWriteByte(void)
{

	// Transmit data

	// Enable transmit
	RS485_TXEN;
	// Send
	//transferDMA_USART1(uint8_t *data, int length)

	/*
	switch(RS485Master.ui8TxState)
	{
		case RS485_M_TX_IDLE:
		{
			// Data to send?
			if(RS485Master.ui8BytesToSend > RS485Master.ui8RcvBufferIndex)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Go to next state
				RS485Master.ui8TxState = RS485_M_TX_SENDING;
				// Reset index
				RS485Master.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Erroreus interrupt. Disable both.
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_M_TX_SENDING:
		{
			// Data to send?
			if(RS485Master.ui8BytesToSend > RS485Master.ui8RcvBufferIndex)
			{
				// Room in buffer?
				if(RS485_TEST_TX_EMPTY)
				{
					// Write one byte
					RS485_WRITE(RS485Master.ui8SendRcvBuffer[RS485Master.ui8RcvBufferIndex]);
					RS485Master.ui8RcvBufferIndex ++;
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Master.ui8TxState = RS485_M_TX_FINISHED;
				// Disable interrupt
				RS485_DISABLE_TX_INT;
				// Enable idle interrupt
				RS485_ENABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_M_TX_FINISHED:
		{
			// Done with sending?
			if(RS485_TEST_TX_IDLE)
			{
				// Yes, enable receive
				RS485_ENABLE_RX;
				// Reset index
				RS485Master.ui8RcvBufferIndex = 0;
				// Disable interrupt
				RS485_DISABLE_TX_IDLE_INT;
				// Go to idle state
				RS485Master.ui8TxState = RS485_TX_IDLE;
			}
			else if(RS485_TEST_TX_EMPTY)
			{
				// This was TX empty interrupt. Disable it.
				RS485_DISABLE_TX_INT;
			}
			break;
		}
		default:
		{
			RS485Master.ui8TxState = RS485_M_TX_IDLE;
			break;
		}
	}*/
	return 0;
}


UInt16 RS485_States_Master(void)
{
	Int16 i = 0;
	switch(RS485Master.ui8MasterState)
	{
		case RS485_M_STATE_IDLE:
		{
			// Check - command waiting?
			if(0 != RS485Master.ui8MasterRequest)
			{
				switch(RS485Master.ui8MasterRequest)
				{
					case RS485_COMMAND_PING:
					{
						// Send ping command
						// Prepare data
						RS485Master.ui8SendRcvBuffer[0] = 0xff;
						RS485Master.ui8SendRcvBuffer[1] = 0xff;
						// ID
						RS485Master.ui8SendRcvBuffer[2] = RS485Master.ui8ReqSlaveAddress;
						RS485Master.ui8SendRcvBuffer[3] = 0x02;
						RS485Master.ui8SendRcvBuffer[4] = 0x01;
						RS485Master.ui8SendRcvBuffer[5] = ~(RS485Master.ui8ReqSlaveAddress + 0x03);
						// Set data length
						RS485Master.ui8BytesToSend = 6;
						RS485Master.ui8RcvBufferIndex = 0;
						// Enable TX interrupts
						RS485_ENABLE_TX_INT;
						// Send first byte
						RS485_MasterWriteByte();
						// Set timeout
						RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
						// Reset request
						RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
						break;
					}
					case RS485_COMMAND_READ:
					{

						break;
					}
					case RS485_COMMAND_WRITE:
					{
						break;
					}
					default:
					{
						break;
					}
				}
			}
			break;
		}
		case RS485_M_STATE_WRITE:
		{


			break;
		}

		case RS485_M_STATE_TORQUE_ON:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x04;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x18;					// Reg address 24
			RS485Master.ui8SendRcvBuffer[6] = 0x01;					// Data to write
			RS485Master.ui8SendRcvBuffer[7] = ~(0x21);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 8;
			RS485Master.ui8RcvBufferIndex = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;


			break;
		}
		case RS485_M_STATE_TORQUE_OFF:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x04;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x18;					// Reg address 24
			RS485Master.ui8SendRcvBuffer[6] = 0x00;					// Data to write
			RS485Master.ui8SendRcvBuffer[7] = ~(0x20);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 8;
			RS485Master.ui8RcvBufferIndex = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;


			break;
		}
		case RS485_M_STATE_READ_ALL:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x04;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_READ;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x00;					// Reg address 0
			RS485Master.ui8SendRcvBuffer[6] = 0x49;					// Data to read - 73 regs
			RS485Master.ui8SendRcvBuffer[7] = ~(0x50);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 8;
			RS485Master.ui8RcvBufferIndex = 0;
			// Set initial register
			RS485Master.RS485Slaves[0].ui8ReadStartAddress = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;
			break;
		}
		case RS485_M_STATE_SET_POS:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x05;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x1E;					// Reg address - goal position
			RS485Master.ui8SendRcvBuffer[6] = RS485Master.RS485Slaves[0].REGS.ui16GoalPosition & 0xFF;
			RS485Master.ui8SendRcvBuffer[7] = (RS485Master.RS485Slaves[0].REGS.ui16GoalPosition >> 8) & 0xFF;
			// Calculate checksum
			RS485Master.ui8SendRcvBuffer[8] = RS485Master.ui8SendRcvBuffer[2];
			for(i = 3; i < 8; i++)
			{
				RS485Master.ui8SendRcvBuffer[8] += RS485Master.ui8SendRcvBuffer[i];
			}
			RS485Master.ui8SendRcvBuffer[8] = ~(RS485Master.ui8SendRcvBuffer[8]);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 9;
			RS485Master.ui8RcvBufferIndex = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;

			break;
		}
		case RS485_M_STATE_SET_SPEED:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x05;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x20;					// Reg address - moving speed
			RS485Master.ui8SendRcvBuffer[6] = RS485Master.RS485Slaves[0].REGS.ui16MovingSpeed & 0xFF;
			RS485Master.ui8SendRcvBuffer[7] = (RS485Master.RS485Slaves[0].REGS.ui16MovingSpeed >> 8) & 0xFF;
			// Calculate checksum
			RS485Master.ui8SendRcvBuffer[8] = RS485Master.ui8SendRcvBuffer[2];
			for(i = 3; i < 8; i++)
			{
				RS485Master.ui8SendRcvBuffer[8] += RS485Master.ui8SendRcvBuffer[i];
			}
			RS485Master.ui8SendRcvBuffer[8] = ~(RS485Master.ui8SendRcvBuffer[8]);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 9;
			RS485Master.ui8RcvBufferIndex = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;
			break;
		}
		case RS485_M_STATE_SET_COMPLIANCE:
		{
			RS485Master.ui8SendRcvBuffer[0] = 0xff;
			RS485Master.ui8SendRcvBuffer[1] = 0xff;
			RS485Master.ui8SendRcvBuffer[2] = 0x01;					// ID
			RS485Master.ui8SendRcvBuffer[3] = 0x07;					// Length
			RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;	// Command
			RS485Master.ui8SendRcvBuffer[5] = 0x26;					// Reg address - compliance
			RS485Master.ui8SendRcvBuffer[6] = RS485Master.RS485Slaves[0].REGS.ui8CWComplianceMargin;
			RS485Master.ui8SendRcvBuffer[7] = RS485Master.RS485Slaves[0].REGS.ui8CCWComplianceMargin;
			RS485Master.ui8SendRcvBuffer[8] = RS485Master.RS485Slaves[0].REGS.ui8CWComplianceSlope;
			RS485Master.ui8SendRcvBuffer[9] = RS485Master.RS485Slaves[0].REGS.ui8CCWComplianceSlope;
			// Calculate checksum
			RS485Master.ui8SendRcvBuffer[10] = RS485Master.ui8SendRcvBuffer[2];
			for(i = 3; i < 10; i++)
			{
				RS485Master.ui8SendRcvBuffer[10] += RS485Master.ui8SendRcvBuffer[i];
			}
			RS485Master.ui8SendRcvBuffer[10] = ~(RS485Master.ui8SendRcvBuffer[10]);				// Checksum
			// Set data length
			RS485Master.ui8BytesToSend = 11;
			RS485Master.ui8RcvBufferIndex = 0;
			// Enable TX interrupts
			RS485_ENABLE_TX_INT;
			// Send first byte
			RS485_MasterWriteByte();
			// Set timeout
			RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
			// Reset request
			RS485Master.ui8MasterState = RS485_M_STATE_REQUEST;
			break;
		}
		case RS485_M_STATE_REQUEST:
		{
			// Wait for response
			// Check timeout
			RS485Master.ui16SlaveTimeout--;
			// Timeout?
			if(0 == RS485Master.ui16SlaveTimeout)
			{
				// Disable interrupts
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
				RS485Master.ui8BytesToSend = 0;
				RS485Master.ui8RcvBufferIndex = 0;
				RS485Master.ui8MasterState = RS485_M_STATE_IDLE;
				RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
			}
			break;
		}
		default:
		{
			// Disable interrupts
			RS485_DISABLE_TX_INT;
			RS485_DISABLE_TX_IDLE_INT;
			RS485Master.ui8BytesToSend = 0;
			RS485Master.ui8RcvBufferIndex = 0;
			RS485Master.ui8MasterState = RS485_M_STATE_IDLE;
			RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
			break;
		}
	}
	return 0;
}

UInt16 RS485_MasterecodeMessage(UInt8 data)
{
	Int16 i = 0;
	switch(RS485Master.ui8RcvState)
	{
		case RS485_M_IDLE:
		{
			if(0xff == data)
			{
				RS485Master.ui8RcvState = RS485_M_WAIT_FOR_SIGNAL;
			}
			break;
		}
		case RS485_M_WAIT_FOR_SIGNAL:
		{
			if(0xff == data)
			{
				RS485Master.ui8RcvState = RS485_M_WAIT_FOR_ID;
			}
			break;
		}
		case RS485_M_WAIT_FOR_ID:
		{
			// We received ID, set pointer
			for(i = 0; i < RS485_NUMSLAVES; i++)
			{
				if(data == RS485Master.RS485Slaves[i].REGS.ui8ID)
				{
					RS485Master.RS485CurrentSlave = &RS485Master.RS485Slaves[i];
					// Store ID to checksum
					RS485Master.RS485CurrentSlave->ui8Checksum = data;
					RS485Master.ui8RcvState = RS485_M_WAIT_FOR_LENGTH;
					i = RS485_NUMSLAVES;
				}
			}
			// Do we have this ID?
			if(RS485_M_WAIT_FOR_ID == RS485Master.ui8RcvState)
			{
				// We failed to get pointer to structure for this ID
				// Go to idle state
				RS485Master.ui8RcvState = RS485_M_IDLE;
			}
			break;
		}
		case RS485_M_WAIT_FOR_LENGTH:
		{
			// Store length
			RS485Master.ui8DataLength = data - 2;
			// Add to checksum
			RS485Master.RS485CurrentSlave->ui8Checksum += data;
			// Next state
			RS485Master.ui8RcvState = RS485_M_WAIT_FOR_INSTR_ERR;
			break;
		}
		case RS485_M_WAIT_FOR_INSTR_ERR:
		{
			RS485Master.RS485CurrentSlave->ui8Status = data;
			// Add to checksum
			RS485Master.RS485CurrentSlave->ui8Checksum += data;
			// Check - data to receive?
			if(0 != RS485Master.ui8DataLength)
			{
				RS485Master.ui8RcvState = RS485_M_WAIT_FOR_PARAMETERS;
			}
			else
			{
				// Go to get checksum
				RS485Master.ui8RcvState = RS485_M_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_M_WAIT_FOR_PARAMETERS:
		{
			// Store data
			RS485Master.RS485CurrentSlave->ui8Data[RS485Master.RS485CurrentSlave->ui8ReadStartAddress] = data;
			// Add to checksum
			RS485Master.RS485CurrentSlave->ui8Checksum += data;
			// Increase address
			RS485Master.RS485CurrentSlave->ui8ReadStartAddress ++;
			// Decrease bytes to receive
			RS485Master.ui8DataLength --;
			// All received?
			if(0 == RS485Master.ui8DataLength)
			{
				// Go to get checksum
				RS485Master.ui8RcvState = RS485_M_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_M_WAIT_FOR_CHECKSUM:
		{
			// Check
			RS485Master.RS485CurrentSlave->ui8Checksum = ~RS485Master.RS485CurrentSlave->ui8Checksum;
			if(data == RS485Master.RS485CurrentSlave->ui8Checksum)
			{
				// Data is good
				RS485Master.RS485CurrentSlave->ui8DataGood = 1;
			}
			else
			{
				// Data is not OK
				RS485Master.RS485CurrentSlave->ui8DataGood = 0;
			}
			break;
		}

		default:
		{
			RS485Master.ui8RcvState = RS485_M_IDLE;
			break;
		}
	}

	// Just put stuff in a array
	RS485Master.ui8SendRcvBuffer[RS485Master.ui8RcvBufferIndex] = data;
	RS485Master.ui8RcvBufferIndex++;
	/*
	switch(RS485Master.ui8RcvState)
	{
		case RS485_M_IDLE:
		{
			// Data = signal?

			break;
		}
		default:
		{
			RS485Master.ui8RcvState = RS485_M_IDLE;
			break;
		}
	}*/

	return 0;
}

// Slave functions
UInt16 RS485_initData(void)
{
	RS485Data.REGS.ui8ID = RS485_ID;
	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	// Set states
	RS485Data.ui8RcvState = RS485_IDLE;
	RS485Data.ui8TxState = RS485_TX_IDLE;
	return 0;
}

UInt16 RS485_writeByte(void)
{
	switch(RS485Data.ui8TxState)
	{
		case RS485_TX_IDLE:
		{
			// Data to send?
			if(RS485Data.ui8BytesToSend > RS485Data.ui8RcvBufferIndex)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Go to next state
				RS485Data.ui8TxState = RS485_TX_SENDING;
				// Reset index
				RS485Data.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Erroreus interrupt. Disable both.
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_TX_SENDING:
		{
			// Data to send?
			if(RS485Data.ui8BytesToSend > RS485Data.ui8RcvBufferIndex)
			{
				// Room in buffer?
				if(RS485_TEST_TX_EMPTY)
				{
					// Write one byte
					RS485_WRITE(RS485Data.ui8SendRcvBuffer[RS485Data.ui8RcvBufferIndex]);
					RS485Data.ui8RcvBufferIndex ++;
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Data.ui8TxState = RS485_TX_FINISHED;
				// Disable interrupt
				RS485_DISABLE_TX_INT;
				// Enable idle interrupt
				RS485_ENABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_TX_FINISHED:
		{
			// Done with sending?
			if(RS485_TEST_TX_IDLE)
			{
				// Yes, enable receive
				RS485_ENABLE_RX;
				// Disable interrupt
				RS485_DISABLE_TX_IDLE_INT;
				// Go to idle state
				RS485Data.ui8TxState = RS485_TX_IDLE;
			}
			else if(RS485_TEST_TX_EMPTY)
			{
				// This was TX empty interrupt. Disable it.
				RS485_DISABLE_TX_INT;
			}
			break;
		}
		default:
		{
			RS485Data.ui8TxState = RS485_TX_IDLE;
			break;
		}
	}

	return 0;
}

UInt16 RS485_States_slave(UInt8 data)
{
	switch(RS485Data.ui8RcvState)
	{
		case RS485_IDLE:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for signal
				RS485Data.ui8RcvState = RS485_WAIT_FOR_SIGNAL;
			}
			break;
		}
		case RS485_WAIT_FOR_SIGNAL:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for ID
				RS485Data.ui8RcvState = RS485_WAIT_FOR_ID;
			}
			else
			{
				RS485Data.ui8RcvState = RS485_IDLE;
			}
			break;
		}
		case RS485_WAIT_FOR_ID:
		{
			// Data == ID?
			if(data == RS485Data.REGS.ui8ID)
			{
				// ID match, wait for data length
				RS485Data.ui8RcvState = RS485_WAIT_FOR_LENGTH;
				// Set checksum to ID
				RS485Data.ui8Checksum = data;
			}
			else
			{
				// Id not mached, go to idle
				RS485Data.ui8RcvState = RS485_IDLE;
			}
			break;
		}
		case RS485_WAIT_FOR_LENGTH:
		{
			// Store how many bytes will follow
			//
			RS485Data.ui8DataLength = data - 2;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Go to wait for instruction
			RS485Data.ui8RcvState = RS485_WAIT_FOR_INSTR_ERR;
			break;
		}
		case RS485_WAIT_FOR_INSTR_ERR:
		{
			// Store instruction
			RS485Data.ui8InstrErr = data;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Go to wait for parameters, if RS485Data.ui8DataLength > 0
			if(0 < RS485Data.ui8DataLength)
			{
				// Waiting for parameters
				RS485Data.ui8RcvState = RS485_WAIT_FOR_PARAMETERS;
				// Reset buffer index
				RS485Data.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Wait for checksum
				RS485Data.ui8RcvState = RS485_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_WAIT_FOR_PARAMETERS:
		{
			// Store parameter
			RS485Data.ui8SendRcvBuffer[RS485Data.ui8RcvBufferIndex] = data;
			RS485Data.ui8RcvBufferIndex ++;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Have all parameters?
			if(RS485Data.ui8DataLength == RS485Data.ui8RcvBufferIndex)
			{
				// Yes, go to receive checksum
				RS485Data.ui8RcvState = RS485_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_WAIT_FOR_CHECKSUM:
		{
			//RS485Data.ui8Checksum = data;
			// Check checksum
			// Calculate
			RS485Data.ui8Checksum = RS485Data.ui8Checksum & 0XFF;
			RS485Data.ui8Checksum = ~RS485Data.ui8Checksum;
			// If checksum is OK, decode data
			if(RS485Data.ui8Checksum == data)
			{
				// Decode received message
				RS485_decodeMessage();
			}
			// Go to idle state
			RS485Data.ui8RcvState = RS485_IDLE;
			break;
		}
		default:
		{
			RS485Data.ui8RcvState = RS485_IDLE;
		}

	}
	return 0;
}

UInt16 RS485_decodeMessage(void)
{
	UInt8 ui8Temp = 0;
	switch(RS485Data.ui8InstrErr)
	{
		case RS485_INSTR_PING:
		{
			break;
		}
		case RS485_INSTR_READ_DATA:
		{
			// Setup data to be transmitted
			// Get address and length
			RS485Data.ui8RWAddress = RS485Data.ui8SendRcvBuffer[0];
			RS485Data.ui8DataLength = RS485Data.ui8SendRcvBuffer[1];
			// Signal
			RS485Data.ui8SendRcvBuffer[0] = 0xFF;
			RS485Data.ui8SendRcvBuffer[1] = 0xFF;
			// ID
			RS485Data.ui8SendRcvBuffer[2] = RS485Data.REGS.ui8ID;
			// Length
			// Error
			RS485Data.ui8SendRcvBuffer[4] = RS485Data.REGS.ui8AlarmLED;
			RS485Data.ui8BytesToSend = 5;
			// Set Length to 2
			RS485Data.ui8SendRcvBuffer[3] = 2;
			// Data
			for(ui8Temp = 0; ui8Temp < RS485Data.ui8DataLength; ui8Temp++)
			{
				// Store byte
				RS485Data.ui8SendRcvBuffer[RS485Data.ui8BytesToSend] = RS485Data.ui8Data[RS485Data.ui8RWAddress + ui8Temp];
				// Increase counter
				RS485Data.ui8BytesToSend ++;
				// Increase Length
				RS485Data.ui8SendRcvBuffer[3] ++;
			}
			// Calculate checksum
			RS485Data.ui8Checksum = 0;
			for(ui8Temp = 2; ui8Temp < RS485Data.ui8BytesToSend; ui8Temp++)
			{
				RS485Data.ui8Checksum += RS485Data.ui8SendRcvBuffer[ui8Temp];
			}
			RS485Data.ui8Checksum = RS485Data.ui8Checksum & 0xFF;
			RS485Data.ui8Checksum = ~RS485Data.ui8Checksum;
			// Store checksum
			RS485Data.ui8SendRcvBuffer[RS485Data.ui8BytesToSend] = RS485Data.ui8Checksum;
			RS485Data.ui8BytesToSend ++;
			// Reset buffer index
			RS485Data.ui8RcvBufferIndex = 0;
			// Enable TX empty interrupt
			RS485_ENABLE_TX_INT;
			break;
		}
	}
	return 0;
}

