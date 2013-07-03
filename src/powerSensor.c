/*
 * powerSensor.c
 *
 *  Created on: Oct 15, 2012
 *      Author: Jure
 */

#include "allinclude.h"

// CRC table
const int crc8_Table[ ] =
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
volatile uint8_t PS_CRC;
volatile uint8_t PS_ReceiveStateVar = 0;
volatile int PS_TIMER = 0;
volatile PS_DataStruct PS_DATA;
volatile int PS_PollTimer = 0;

void PS_Timer(void)
{
	PS_TIMER++;
	if(PS_TIMER > PS_MAXTIME)
	{
		PS_TIMER = PS_MAXTIME;
	}
	// Check if we are waiting to receive data
	if(PS_WAITINGDATA)
	{
		if(PS_TIMER > PS_TIMEOUT)
		{
			PS_ReceiveStateVar = PS_IDLE;
			PS_WAITINGDATA = 0;
			PSBUSY = 0;
        	// Mark sensor not OK
        	SCR2 = SCR2 & ~SCR2_POWEROK;
		}
	}
	if(PS_ReceiveStateVar != PS_IDLE)
	{
		if(PS_TIMER > PS_TIMEOUT)
		{
			PS_ReceiveStateVar = PS_IDLE;
			PS_WAITINGDATA = 0;
			PSBUSY = 0;
        	// Mark sensor not OK
        	SCR2 = SCR2 & ~SCR2_POWEROK;
		}
	}
}

void PSRequestData(void)
{
	uint32_t timeout = 100000;
	PSBUSY = 1;
	PS_WAITINGDATA = 1;
	PS_TIMER = 100;
	USART_SendData(USART1, 0x57);
	// Wait
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TC))
	{
		timeout--;
		if(timeout == 0) break;
	}
	Delaynus(1000);
	// Send CRC
	USART_SendData(USART1, 0x6d);
}

void PSSetI0(void)
{
	uint32_t timeout = 100000;
	USART_SendData(USART1, 0x58);
	// Wait
	Delaynus(1000);
	// Wait
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TC))
	{
		timeout--;
		if(timeout == 0) break;
	}
	// Send CRC
	USART_SendData(USART1, 0x2c);
	// Wait for finish
	timeout = 100000;
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TC))
	{
		timeout--;
		if(timeout == 0) break;
	}
}

void PSReset(void)
{
	uint32_t timeout = 100000;
	PSBUSY = 1;
	USART_SendData(USART1, 0x59);
	// Wait
	Delaynus(1000);
	// Wait
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TC))
	{
		timeout--;
		if(timeout == 0) break;
	}
	// Send CRC
	USART_SendData(USART1, 0x72);
	// Wait for finish
	timeout = 100000;
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TC))
	{
		timeout--;
		if(timeout == 0) break;
	}
}

void processPSData(void)
{
	CURRENT = PS_DATA.data.Current;
	MAH = PS_DATA.data.mAh;
	VOLTAGE = PS_DATA.data.Voltage;
	T1 = PS_DATA.data.T_1;
	T2 = PS_DATA.data.T_2;
	T3 = PS_DATA.data.T_3;
	PS_ReceiveStateVar = PS_IDLE;
}

void PowerSensorCommProcess(uint8_t data)
{
	// Calculate CRC
	PS_CalculateCRC(data);
	if(PS_ReceiveStateVar == PS_IDLE)
	{
		#ifdef PS_ENABLETIMEOUT
		if(PS_TIMER < PS_3CHAR)
		{
			data = 0;
		}
		#endif
		if((data & 0xF0) == PS_SENSORID)
		{
			// Initialize CRC
			PS_CRC = 0xFF;
			// Calculate CRC
			PS_CalculateCRC(data);
			// Store data
			PS_DATA.bytes.IDFCODE = data;
			PS_ReceiveStateVar = PS_GETCURRENTLOW;
			// Mark not waiting for data anymore
			PS_WAITINGDATA = 0;
		}
	}
	else
	{
		#ifdef PS_ENABLETIMEOUT
		if(PS_TIMER > PS_1CHAR)
		{
			data = 0;
			PS_ReceiveStateVar = PS_IDLE;
		}
		#endif
		switch(PS_ReceiveStateVar)
		{
			case PS_GETCURRENTLOW:
			{
				PS_DATA.bytes.CurrentLow = data;
				PS_ReceiveStateVar = PS_GETCURRENTHI;
				break;
			}
			case PS_GETCURRENTHI:
			{
				PS_DATA.bytes.CurrentHi = data;
				PS_ReceiveStateVar = PS_GETMAHLOW;
				break;
			}
			case PS_GETMAHLOW:
			{
				PS_DATA.bytes.mAhLow = data;
				PS_ReceiveStateVar = PS_GETMAHHI;
				break;
			}
			case PS_GETMAHHI:
			{
				PS_DATA.bytes.mAhHi = data;
				PS_ReceiveStateVar = PS_GETVOLTAGELOW;
				break;
			}
			case PS_GETVOLTAGELOW:
			{
				PS_DATA.bytes.VoltageLow = data;
				PS_ReceiveStateVar = PS_GETVOLTAGEHI;
				break;
			}
			case PS_GETVOLTAGEHI:
			{
				PS_DATA.bytes.VoltageHi = data;
				PS_ReceiveStateVar = PS_GETTEMP1LOW;
				break;
			}
			case PS_GETTEMP1LOW:
			{
				PS_DATA.bytes.T1Low = data;
				PS_ReceiveStateVar = PS_GETTEMP1HI;
				break;
			}
			case PS_GETTEMP1HI:
			{
				PS_DATA.bytes.T1Hi = data;
				PS_ReceiveStateVar = PS_GETTEMP2LOW;
				break;
			}
			case PS_GETTEMP2LOW:
			{
				PS_DATA.bytes.T2Low = data;
				PS_ReceiveStateVar = PS_GETTEMP2HI;
				break;
			}
			case PS_GETTEMP2HI:
			{
				PS_DATA.bytes.T2Hi = data;
				PS_ReceiveStateVar = PS_GETTEMP3LOW;
				break;
			}
			case PS_GETTEMP3LOW:
			{
				PS_DATA.bytes.T3Low = data;
				PS_ReceiveStateVar = PS_GETTEMP3HI;
				break;
			}
			case PS_GETTEMP3HI:
			{
				PS_DATA.bytes.T3Hi = data;
				PS_ReceiveStateVar = PS_GETCRC;
				break;
			}
			case PS_GETCRC:
			{
				PS_DATA.bytes.PSCRC = data;
				// Check CRC
				if(PS_CRC == 0)
				{
					// CRC is OK
					PS_ReceiveStateVar = PS_DATAREADY;
				}
				else
				{
					PS_ReceiveStateVar = PS_IDLE;
				}
				break;
			}
			case PS_DATAREADY:
			{
				break;
			}
			default:
			{
				PS_ReceiveStateVar = PS_IDLE;
				break;
			}
		}
	}
#ifdef PS_ENABLETIMEOUT
	// Reset timeout timer
	PS_TIMER = 0;
#endif
}

void PS_CalculateCRC(uint8_t data)
{
	PS_CRC = (uint8_t)crc8_Table[PS_CRC ^ data];
}

