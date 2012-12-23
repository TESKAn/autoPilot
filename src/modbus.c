/*
 * modbus.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#include "allinclude.h"

// Variables
volatile int MODBUS_ReceiveState = 0;
volatile unsigned char ucMBCRCLOW = 0;
volatile unsigned char ucMBCRCHI = 0;
volatile int MODBUS_Timeout = 0;
// MODBUS registers
uint16_t MODBUSReg[MB_TOTALREGISTERS];
// Var holds data
Typedef_mbd MODBUSData;
// MODBUS temp reg variable
volatile uint16_t MODBUSTempReg = 0;

// CRC tables
/* Table of CRC values for high–order byte */
static unsigned char ucCRCTableHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

/* Table of CRC values for low–order byte */
static unsigned char ucCRCTableLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;

// Include _all_ functions needed for modbus, _NO_ hardware functions

// Function ModBus timeout
void MODBUS_Timer(void)
{
	// Call every 1 millisecond
#ifdef MB_USETIMEOUTS
	MODBUS_Timeout++;
	if(MODBUS_Timeout > MB_MAXTIME)
	{
		MODBUS_Timeout = MB_MAXTIME;
	}
	if(MODBUS_ReceiveState != MB_IDLE)
	{
		if(MODBUS_Timeout > MB_TIMEOUT)
		{
			MODBUS_ReceiveState = MB_IDLE;							//set state
		}
	}
#endif
}

// Function to process data once it is received
void MODBUS_ExecuteFunction()
{
	int i = 0;
	int writeDataIndex = 0;
	int responseByteCount = 0;
	unsigned int uiTemp = 0;
	// Check function
	switch(MODBUSData.data.cFunctionCode)
	{
		case MBRWMULTIPLEREGISTERS:
		{
			// Read/write registers
			// First write all registers
			// Set index to start from 0
			writeDataIndex = 0;
			for(i=MODBUSData.data.uiWriteStartingAddress; i < MODBUSData.data.uiWriteStartingAddress + MODBUSData.data.uiDataCount; i++)
			{
				// Check address overflow
				if((i >= 0) && (i < MB_TOTALREGISTERS))
				{
					//*MODBUS_Pointers[i] = MODBUSData.data.uiData[writeDataIndex];

					MODBUSReg[i] = MODBUSData.data.uiData[writeDataIndex];
					// Increase data index
					writeDataIndex++;
				}
			}
			// Second read data from registers
			// First setup response
			responseByteCount = 0;
			// Store slave ID
			MODBUSData.bytes.cdata[0] = MB_SLAVEID;
			// Store function code
			MODBUSData.bytes.cdata[1] = MBRWMULTIPLEREGISTERS;
			// Store data quantity
			MODBUSData.bytes.cdata[2] = MODBUSData.data.uiQuantToRead * 2;
			// Store how much data we have
			responseByteCount = 3;
			// Store data
			for(i=MODBUSData.data.uiReadStartingAddress; i < MODBUSData.data.uiReadStartingAddress + MODBUSData.data.uiQuantToRead; i++)
			{
				// Check address overflow
				if((i >= 0) && (i < MB_TOTALREGISTERS))
				{
					//uiTemp = *MODBUS_Pointers[i];
					uiTemp = MODBUSReg[i];
					// Store data to buffer
					MODBUSData.bytes.cdata[responseByteCount] = (uiTemp >> 8) & 0x00FF;
					responseByteCount++;
					MODBUSData.bytes.cdata[responseByteCount] = uiTemp & 0x00FF;
					responseByteCount++;
				}
			}
			// Store number of bytes
			MODBUSData.bytes.uiDataCount = responseByteCount;
			// Calculate CRC and add it to message
			crcOnMessage(responseByteCount);
			break;
		}
	}

}

void MODBUS_ProcessData(unsigned int data)
{
	//calculate CRC
	crcOnByte(data);
	// Check if IDLE
	if(MODBUS_ReceiveState == MB_IDLE)
	{
		// Check enough time has passed
		#ifdef MB_USETIMEOUTS
		if(MODBUS_Timeout < MB_3CHAR)
		{
			// If not, reset data to something else than slave ID
			data = MB_SLAVEID + 1;
		}
		#endif
		// Check if we received ID
		if(data == MB_SLAVEID)
		{
			// If OK, initialize CRC
			ucMBCRCHI = 0xFF;
			ucMBCRCLOW = 0xFF;
			// Calculate first byte CRC
			crcOnByte(data);
			// Store data
			MODBUSData.data.cSlaveID = (char)data;
			// Go to next step
			MODBUS_ReceiveState = MB_RECEIVINGFCODE;
		}
	}
	else
	{
		// Check new byte arrived fast enough
		#ifdef MB_USETIMEOUTS
		if(MODBUS_Timeout > MB_1CHAR)
		{
			// If not, reset data to something else than slave ID
			data = MB_SLAVEID + 1;
			// And reset state to idle
			MODBUS_ReceiveState = MB_IDLE;
		}
		#endif
		switch(MODBUS_ReceiveState)
		{
			case MB_RECEIVINGFCODE:
			{
				MODBUSData.bytes.cFunctionCode = (char)data;
				switch(MODBUSData.bytes.cFunctionCode)
				{
					case MBRWMULTIPLEREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGRSTARTADDRESS_HI;
						break;
					}
					case MBWRITESINGLEREGISTER:
					{
						MODBUS_ReceiveState = MB_RECEIVINGWRITESTART_HI;
						break;
					}
					case MBREADHOLDINGREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGRSTARTADDRESS_HI;
						break;
					}
					case MBWRITEMULTIPLEREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGWRITESTART_HI;
						break;
					}
				}
				break;
			}
			case MB_RECEIVINGRSTARTADDRESS_HI:
			{
				MODBUSData.bytes.uiReadStartingAddressHi = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGRSTARTADDRESS_LO;
				break;
			}
			case MB_RECEIVINGRSTARTADDRESS_LO:
			{
				MODBUSData.bytes.uiReadStartingAddressLo  = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGQREAD_HI;
				break;
			}
			case MB_RECEIVINGQREAD_HI:
			{
				MODBUSData.bytes.uiQuantToReadHi  = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGQREAD_LO;
				break;
			}
			case MB_RECEIVINGQREAD_LO:
			{
				MODBUSData.bytes.uiQuantToReadLo = (unsigned char)data;
				switch(MODBUSData.bytes.cFunctionCode)
				{
					case MBRWMULTIPLEREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGWRITESTART_HI;
						break;
					}
					case MBREADHOLDINGREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGCRC_HI;
						break;
					}
				}
				break;
			}
			case MB_RECEIVINGWRITESTART_HI:
			{
				MODBUSData.bytes.uiWriteStartingAddressHi = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGWRITESTART_LO;
				break;
			}
			case MB_RECEIVINGWRITESTART_LO:
			{
				MODBUSData.bytes.uiWriteStartingAddressLo = (unsigned char)data;
				switch(MODBUSData.bytes.cFunctionCode)
				{
					case MBRWMULTIPLEREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGQWRITE_HI;
						break;
					}
					case MBWRITESINGLEREGISTER:
					{
						MODBUSData.data.uiQuantToWrite = 1;
						MODBUSData.bytes.ucWriteByteCount = 2;
						MODBUS_ReceiveState = MB_RECEIVINGDATA_HI;
						break;
					}
					case MBWRITEMULTIPLEREGISTERS:
					{
						MODBUS_ReceiveState = MB_RECEIVINGQWRITE_HI;
						break;
					}
				}
				break;
			}
			case MB_RECEIVINGQWRITE_HI:
			{
				MODBUSData.bytes.uiQuantToWriteHi = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGQWRITE_LO;
				break;
			}
			case MB_RECEIVINGQWRITE_LO:
			{
				MODBUSData.bytes.uiQuantToWriteLo = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGBYTECOUNT;
				break;
			}
			case MB_RECEIVINGBYTECOUNT:
			{
				MODBUSData.bytes.ucWriteByteCount = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGDATA_HI;
				MODBUSData.data.uiDataIndex = 0;
				MODBUSData.bytes.uiDataCount = 0;
				break;
			}
			case MB_RECEIVINGDATA_HI:
			{
				MODBUSData.bytes.cdata[MODBUSData.bytes.uiDataIndex+1] = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGDATA_LO;
				break;
			}
			case MB_RECEIVINGDATA_LO:
			{
				MODBUSData.bytes.cdata[MODBUSData.bytes.uiDataIndex] = (unsigned char)data;
				MODBUSData.bytes.uiDataIndex += 2;
				MODBUSData.bytes.uiDataCount++;
				if(MODBUSData.bytes.uiDataIndex == MODBUSData.bytes.ucWriteByteCount)
				{
					// If all data received
					MODBUS_ReceiveState = MB_RECEIVINGCRC_HI;
				}
				else
				{
					MODBUS_ReceiveState = MB_RECEIVINGDATA_HI;
				}
				break;
			}
			case MB_RECEIVINGCRC_HI:
			{
				MODBUSData.bytes.uiCRCHi = (unsigned char)data;
				MODBUS_ReceiveState = MB_RECEIVINGCRC_LO;
				break;
			}
			case MB_RECEIVINGCRC_LO:
			{
				MODBUSData.bytes.uiCRCLo = (unsigned char)data;
				MODBUS_ReceiveState = MB_DATAREADY;
				if((ucMBCRCHI == 0)&&(ucMBCRCLOW == 0))
				{
					MODBUSData.data.ucDataReady = 1;
				}
				break;
			}
			case MB_DATAREADY:
			{
				break;
			}
			default:
			{
				MODBUS_ReceiveState = MB_IDLE;
				break;
			}
		}
	}
	// Reset timer
	MODBUS_Timeout = 0;
}

// Function slave calculate CRC on received byte
void crcOnByte(unsigned int ucMsgByte)
{
	unsigned int uiIndex = 0;
	uiIndex = (unsigned int)(ucMBCRCHI^ucMsgByte) ; //calculate the CRC
	ucMBCRCHI = (unsigned char) (ucMBCRCLOW ^ ucCRCTableHi[uiIndex]) ;
	ucMBCRCLOW = ucCRCTableLo[uiIndex] ;
}

// Function slave calculate CRC on message
void crcOnMessage(unsigned int uiDataLen)
{
	int i = 0;
	unsigned int uiIndex = 0;
	ucMBCRCHI = 0xFF ;  	// high byte of CRC initialized
	ucMBCRCLOW = 0xFF ;  // low byte of CRC initialized

	uiIndex = 0;
	for(i=0;i < uiDataLen;i++)
	{
		uiIndex = (unsigned int)(ucMBCRCHI^MODBUSData.bytes.cdata[i]) ; // calculate the CRC
		ucMBCRCHI = (unsigned char) (ucMBCRCLOW ^ ucCRCTableHi[uiIndex]) ;
		ucMBCRCLOW = ucCRCTableLo[uiIndex] ;
	}
	MODBUSData.bytes.cdata[MODBUSData.bytes.uiDataCount] = ucMBCRCHI;
	MODBUSData.bytes.uiDataCount ++;
	MODBUSData.bytes.cdata[MODBUSData.bytes.uiDataCount] = ucMBCRCLOW;
	MODBUSData.bytes.uiDataCount ++;
}
