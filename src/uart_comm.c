/*
 * uart_comm.c
 *
 *  Created on: 27. nov. 2015
 *      Author: Jure
 */
#include "allinclude.h"



// CRC table
const int crc8_ucTable[ ] =
{
     0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
   157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
    35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
   190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
    70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
   219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
   101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
   248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
   140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
    17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
   175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
    50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
   202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
    87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
   233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
   116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
} ;

// Variables
uint8_t UART2_transmittBuffer[1024];
uint8_t UART2_RecBuffer[16];
uint8_t UART2_Transferring = 0;
volatile uint8_t UART_CRC;
uint8_t UART2_RcvdBytes = 0;
uint16_t UART2_TimeoutCounter = 0;
uint16_t UART2_TimeoutTime = 65500;
// Var ID that we are receiving
uint16_t UART2_RcvingVar = 0;

RING_BUFFER UARTBuffer;

CONVERTNUM UART_Conversion;

void UART_CalculateCRC(uint8_t data)
{
	UART_CRC = (uint8_t)crc8_ucTable[UART_CRC ^ data];
}

uint8_t UART_BufCRC(uint8_t *data, int16_t bytes)
{
	uint8_t crc = 0xff;
	int i = 0;
	for(i = 0; i < bytes; i++)
	{
		crc = (uint8_t)crc8_ucTable[crc ^ data[i]];
	}
	return crc;
}

void UART_Init()
{
	RB_Init(&UARTBuffer, UART2_transmittBuffer, 1024);
	UART2_RcvdBytes = 0;
	UART2_RcvingVar = 0;
	UART_CRC = 0xff;
}

int32_t UART_RcvData(uint8_t data)
{
	// Reset timeout
	UART2_TimeoutCounter = 0;
	// Calc CRC on this data
	UART_CalculateCRC(data);
	// Store it
	UART2_RecBuffer[UART2_RcvdBytes] = data;
	UART2_RcvdBytes++;
	// Check for header
	if(2 > UART2_RcvdBytes)
	{
		// If data is not 0xff, reset receive process
		if(0xff != data)
		{
			UART2_RcvdBytes = 0;
			UART2_RcvingVar = 0;
			UART_CRC = 0xff;
		}
	}
	// Check data pos
	else if(4 == UART2_RcvdBytes)
	{
		// We have var ID. Store it.
		UART_Conversion.ch[0] = UART2_RecBuffer[2];
		UART_Conversion.ch[1] = UART2_RecBuffer[3];
		UART2_RcvingVar = UART_Conversion.i16[0];
	}
	else if(UART2_RcvdBytes > 4)
	{

		switch(UART2_RcvingVar)
		{
			case 0:
			case 1:
			{
				UART2_RcvdBytes = 0;
				UART2_RcvingVar = 0;
				UART_CRC = 0xff;
				break;
			}
			case VAR_MAIN_LOOP_STATE:
			{
				// mainLoopState, uint16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						mainLoopState = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_RS485_SERVO_POSITION:
			{
				// servo position, uint16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						servoMovePosition = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_MOTOR_FR_RPM:
			{
				// Motor RPM, float32
				if(UART2_RcvdBytes == 9)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						UART_Conversion.ch[2] = UART2_RecBuffer[6];
						UART_Conversion.ch[3] = UART2_RecBuffer[7];
						motorFRSpeed = UART_Conversion.f32[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_RREADRS485DATA:
			{
				// readRS485Data, int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						readRS485Data = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_RS485COMMAND:
			{
				// RS485 data, uint32
				if(UART2_RcvdBytes == 9)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						UART_Conversion.ch[2] = UART2_RecBuffer[6];
						UART_Conversion.ch[3] = UART2_RecBuffer[7];
						RS485ExecuteCommand.ui32Packed = UART_Conversion.ui32[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_PWMOUT_8:
			{
				// int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						RCData.ch[7].PWMOUT = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_PWMOUT_9:
			{
				// int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						RCData.ch[8].PWMOUT = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_PWMOUT_10:
			{
				// int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						RCData.ch[9].PWMOUT = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_PWMOUT_11:
			{
				// int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						RCData.ch[10].PWMOUT = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_PWMOUT_12:
			{
				// int16_t
				if(UART2_RcvdBytes == 7)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						RCData.ch[11].PWMOUT = UART_Conversion.i16[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			case VAR_UI32TESTVAR:
			{
				if(UART2_RcvdBytes == 9)
				{
					// Check CRC
					if(0 == UART_CRC)
					{
						// Store var
						UART_Conversion.ch[0] = UART2_RecBuffer[4];
						UART_Conversion.ch[1] = UART2_RecBuffer[5];
						UART_Conversion.ch[2] = UART2_RecBuffer[6];
						UART_Conversion.ch[3] = UART2_RecBuffer[7];
						ui32TestVar = UART_Conversion.ui32[0];
					}
					UART2_RcvdBytes = 0;
					UART2_RcvingVar = 0;
					UART_CRC = 0xff;
				}
				break;
			}
			default:
			{
				UART2_RcvdBytes = 0;
				UART2_RcvingVar = 0;
				UART_CRC = 0xff;
				break;
			}
		}
	}
	return 0;
}

void UART_Timeout()
{
	UART2_TimeoutCounter++;
	// Timeout?
	if(UART2_TimeoutCounter > UART2_TimeoutTime)
	{
		UART2_TimeoutCounter = UART2_TimeoutTime;
		UART2_RcvdBytes = 0;
		UART2_RcvingVar = 0;
		UART_CRC = 0xff;
	}
}

int32_t UART_SendBuffer()
{
#ifndef USE_FREEMASTER
	int bytes = 0;

	// If UART2 is not active
	if(!UART2_Transferring)
	{
		// If buffer is half full
		if(512 < UARTBuffer.count)
		{
			// Copy data to transmit buffer
			bytes = UART_CopyToTransmitBuf();
			// Send
			transferDMA_USART2(UART2Buffer, bytes);
			UART2_Transferring = 1;
			return 0;
		}
		return -1;
	}
#endif
	return -1;

}

int32_t UART_BufCount()
{
	return  UARTBuffer.count;
}

int UART_CopyToTransmitBuf()
{
	int numBytes = 0;
	while(0 != UARTBuffer.count)
	{
		UART2Buffer[numBytes] = RB_pop(&UARTBuffer);
		numBytes++;
	}
	return numBytes;
}

int32_t UART_QueueMessagei16(int16_t var, int16_t data)
{
	int i = 0;
	uint8_t messageBuffer[7];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	UART_Conversion.i16[1] = data;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];

	messageBuffer[4] = UART_Conversion.ch[2];
	messageBuffer[5] = UART_Conversion.ch[3];
	messageBuffer[6] = UART_BufCRC(messageBuffer, 6);
	for(i = 0; i < 7; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessagei32(int16_t var, int32_t data)
{
	int i = 0;
	uint8_t messageBuffer[9];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];
	UART_Conversion.i32[0] = data;
	messageBuffer[4] = UART_Conversion.ch[0];
	messageBuffer[5] = UART_Conversion.ch[1];
	messageBuffer[6] = UART_Conversion.ch[2];
	messageBuffer[7] = UART_Conversion.ch[3];
	messageBuffer[8] = UART_BufCRC(messageBuffer, 8);
	for(i = 0; i < 9; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessagef(int16_t var, float data)
{
	int i = 0;
	uint8_t messageBuffer[9];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];
	UART_Conversion.f32[0] = data;
	messageBuffer[4] = UART_Conversion.ch[0];
	messageBuffer[5] = UART_Conversion.ch[1];
	messageBuffer[6] = UART_Conversion.ch[2];
	messageBuffer[7] = UART_Conversion.ch[3];
	messageBuffer[8] = UART_BufCRC(messageBuffer, 8);
	for(i = 0; i < 9; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessageui16(int16_t var, uint16_t data)
{
	int i = 0;
	uint8_t messageBuffer[7];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	UART_Conversion.ui16[1] = data;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];

	messageBuffer[4] = UART_Conversion.ch[2];
	messageBuffer[5] = UART_Conversion.ch[3];
	messageBuffer[6] = UART_BufCRC(messageBuffer, 6);
	for(i = 0; i < 7; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessageui32(int16_t var, uint32_t data)
{
	int i = 0;
	uint8_t messageBuffer[9];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];
	UART_Conversion.ui32[0] = data;
	messageBuffer[4] = UART_Conversion.ch[0];
	messageBuffer[5] = UART_Conversion.ch[1];
	messageBuffer[6] = UART_Conversion.ch[2];
	messageBuffer[7] = UART_Conversion.ch[3];
	messageBuffer[8] = UART_BufCRC(messageBuffer, 8);
	for(i = 0; i < 9; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessagei8(int16_t var, int8_t data)
{
	int i = 0;
	uint8_t messageBuffer[7];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	UART_Conversion.i8[2] = data;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];

	messageBuffer[4] = UART_Conversion.ch[2];
	messageBuffer[5] = UART_BufCRC(messageBuffer, 5);
	for(i = 0; i < 6; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

int32_t UART_QueueMessageui8(int16_t var, uint8_t data)
{
	int i = 0;
	uint8_t messageBuffer[7];
	messageBuffer[0] = 0xff;
	messageBuffer[1] = 0xff;
	UART_Conversion.i16[0] = var;
	messageBuffer[2] = UART_Conversion.ch[0];
	messageBuffer[3] = UART_Conversion.ch[1];

	messageBuffer[4] = data;
	messageBuffer[5] = UART_BufCRC(messageBuffer, 5);
	for(i = 0; i < 6; i++)
	{
		RB_push(&UARTBuffer, messageBuffer[i]);
	}
	// If UART is not busy, send data
	if(!UART2_Transferring)
	{
		UART_SendBuffer();
	}
	return 0;
}

