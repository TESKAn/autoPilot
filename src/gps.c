/*
 * gps.c
 *
 *  Created on: Oct 21, 2012
 *      Author: Jure
 */

#include "allinclude.h"
uint8_t GPS_DataBuffer[64];
volatile uint16_t GPS_RECDATA[32];
volatile uint8_t GPS_Digits[16];
volatile uint8_t GPS_ProcesState = 0;
volatile uint8_t GPS_NextData = 0;
volatile uint8_t GPS_Checksum = 0;
volatile uint8_t GPS_Checksum_Save = 0;
volatile uint8_t GPS_ReceivedChecksum[2];
volatile uint8_t GPS_ChecksumCounter = 0;
volatile uint16_t GPS_DataTemp = 0;
volatile uint8_t GPS_DataTemp_Count = 0;
volatile uint8_t GPS_Digits_Count = 0;
// Flag variable
volatile Flag GPSFlag;

void GPSStopOutput(void)
{
	int numbytes = 0;
	int i = 0;
	uint8_t checksum = 0;
	// Mark GPS is sending data
	GPS_SENDING = 1;
	// Fill data
	GPS_DataBuffer[0] = '$';	// Preamble, $ sign
	GPS_DataBuffer[1] = 'P';
	GPS_DataBuffer[2] = 'M';
	GPS_DataBuffer[3] = 'T';
	GPS_DataBuffer[4] = 'K';
	GPS_DataBuffer[5] = '3';
	GPS_DataBuffer[6] = '1';
	GPS_DataBuffer[7] = '4';
	numbytes = 8;
	for(i = 0; i < 19; i++)
	{
		GPS_DataBuffer[numbytes] = ',';
		GPS_DataBuffer[numbytes + 1] = '0';
		numbytes += 2;
	}
	GPS_DataBuffer[numbytes] = '*';
	numbytes++;
	checksum = GPS_CalculateChecksum(GPS_DataBuffer, 1, 46);
	// Convert number to ASCII
	GPS_DataBuffer[numbytes] = GPS_GetChar(checksum, SET);
	numbytes++;
	GPS_DataBuffer[numbytes] = GPS_GetChar(checksum, RESET);
	numbytes++;
	GPS_DataBuffer[numbytes] = 0x0d;	// ASCII <CR>
	numbytes++;
	GPS_DataBuffer[numbytes] = 0x0a;	// ASCII <LF>
	numbytes++;

	// Send to DMA
	transferDMA_USART3(GPS_DataBuffer, numbytes);
}

void GPSSetDataOutput(void)
{
	int numbytes = 0;
	int i = 0;
	uint32_t GPSTime = 0;
	uint8_t checksum = 0;
	// Mark GPS is sending data
	GPS_SENDING = 1;
	// Fill data
	GPS_DataBuffer[0] = '$';	// Preamble, $ sign
	GPS_DataBuffer[1] = 'P';
	GPS_DataBuffer[2] = 'M';
	GPS_DataBuffer[3] = 'T';
	GPS_DataBuffer[4] = 'K';
	GPS_DataBuffer[5] = '3';
	GPS_DataBuffer[6] = '1';
	GPS_DataBuffer[7] = '4';
	numbytes = 8;
	for(i = 0; i < 19; i++)
	{
		GPS_DataBuffer[numbytes] = ',';
		GPS_DataBuffer[numbytes + 1] = '0';
		numbytes += 2;
	}
	// Set GGA to 1
	GPS_DataBuffer[15] = '1';
	// Set RMC to 1
	GPS_DataBuffer[11] = '1';
	GPS_DataBuffer[numbytes] = '*';
	numbytes++;
	checksum = GPS_CalculateChecksum(GPS_DataBuffer, 1, 46);
	// Convert number to ASCII
	GPS_DataBuffer[numbytes] = GPS_GetChar(checksum, SET);
	numbytes++;
	GPS_DataBuffer[numbytes] = GPS_GetChar(checksum, RESET);
	numbytes++;
	GPS_DataBuffer[numbytes] = 0x0d;	// ASCII <CR>
	numbytes++;
	GPS_DataBuffer[numbytes] = 0x0a;	// ASCII <LF>
	numbytes++;

	// Send to DMA
	transferDMA_USART3(GPS_DataBuffer, numbytes);

	// Wait end of transfer
	GPSTime = systemTime;
	while(GPS_SENDING != 0)
	{
		// If waiting time is larger than GPS_TIMEOUT, break.
		if((systemTime - GPSTime)> GPS_IMEOUT)
		{
			break;
		}
	}
}

uint8_t GPS_GetChar(uint8_t data, FlagStatus part)
{
	uint8_t character = 0;
	// Part = SET -> get upper 4 bits, RESET -> get lower 4 bits
	if(part)
	{
		// Upper 4 bits
		character = (data >> 4) & 0x0F;
	}
	else
	{
		character = data & 0x0F;
	}
	if(character < 10)
	{
		// Result = x + 48
		character = character + 48;
	}
	else
	{
		// Result = x -10 + 65 = x + 55
		character = character + 55;
	}
	return character;
}

uint8_t GPS_CalculateChecksum(uint8_t *data, int startByte, int dataCount)
{
	int i = 0;
	uint8_t checksum = GPS_DataBuffer[startByte];
	for(i = startByte + 1; i < dataCount; i++)
	{
		checksum = checksum ^ GPS_DataBuffer[i];
	}
	return checksum;
}

uint8_t GPS_CharToByte(uint8_t data)
{
	// Data is a number 0 - 9 in ASCII
	// To get value, remove 48 from data
	return data - 48;
}

void GPS_ReceiveProcess(uint8_t data)
{
	uint16_t temp = 0;
	// Check that we are not sending data. If we are, do not mess with the buffer
	if(!GPS_SENDING)
	{
		switch(GPS_ProcesState)
		{
			case GPS_WAITINGSTART:
			{
				if(data == '$')
				{
					GPS_ProcesState = GPS_RECEIVE_NUMBER;
					GPS_NextData = GPS_NEXTDATA_DATACODE;
					// Go to next data
					GPS_Digits[0] = 0;
					GPS_Digits[1] = 0;
					GPS_Digits_Count = 0;
					GPSFLAG_CR_RECEIVED = 0;
					GPSFLAG_CHECKSUM_RESET = 1;
					// Mark new data time
					ahrs_data.GPSData.dataStartTime = systemTime;
					// Store rotation matrix
					ahrs_data.GPSReference = ahrs_data.rotationMatrix;
				}
				break;
			}
			case GPS_RECEIVE_NUMBER:
			{
				if(data == ',')
				{
					// Store data and go to next data
					switch(GPS_NextData)
					{
						case GPS_NEXTDATA_DATACODE:
						{
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_TIME;
							GPS_Digits_Count = 0;
							if((GPS_Digits[2] == 'G') && (GPS_Digits[3] == 'G') && (GPS_Digits[4] == 'A'))
							{
								GPSFLAG_GGA = 1;
								GPSFLAG_RMC = 0;
							}
							else if((GPS_Digits[2] == 'R') && (GPS_Digits[3] == 'M') && (GPS_Digits[4] == 'C'))
							{
								GPSFLAG_GGA = 0;
								GPSFLAG_RMC = 1;
							}
							else
							{
								// Else sentence not supported
								GPS_NextData = GPS_NEXTDATA_VOID;
								GPS_ProcesState = GPS_WAITINGSTART;
								GPS_Digits_Count = 0;
							}
							break;
						}
						case GPS_NEXTDATA_STATUS:
						{
							// A=valid, V=invalid
							if(GPS_Digits[0] == 'V')
							{
								// GPS_VALID bit 15: 0=invalid, 1=valid
								GPS_VALID_T = GPS_VALID_T & ~0x8000;
							}
							else
							{
								GPS_VALID_T = GPS_VALID_T | 0x8000;
							}
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_LATITUDE;
							GPS_Digits_Count = 0;
							break;
						}
						// Case data received N/S
						case GPS_NEXTDATA_NS:
						{
							// GPS_NS_EW, bit0: N=0,S=1; bit1:E=0,W=1
							if(GPS_Digits[0] == 'N')
							{
								GPS_NS_EW_T = GPS_NS_EW_T & ~0x0001;
							}
							else
							{
								GPS_NS_EW_T = GPS_NS_EW_T | 0x0001;
							}
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_LONGITUDE;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_EW:
						{
							// GPS_NS_EW, bit0: N=0,S=1; bit1:E=0,W=1
							if(GPS_Digits[0] == 'E')
							{
								GPS_NS_EW_T = GPS_NS_EW_T & ~0x0002;
							}
							else
							{
								GPS_NS_EW_T = GPS_NS_EW_T | 0x0002;
							}
							// Go to next data
							if(GPSFLAG_GGA)
							{
								GPS_NextData = GPS_NEXTDATA_FIXVALID;
							}
							else if(GPSFLAG_RMC)
							{
								GPS_NextData = GPS_NEXTDATA_SPEED;
							}
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_FIXVALID:
						{
							// GPS_VALID bits 0 - 4
							// Clear lower 4 bits
							GPS_VALID_T = GPS_VALID_T & 0xFFF0;
							GPS_VALID_T = GPS_VALID_T | ((GPS_Digits[0] - 48) & 0x0F);
							// Go to next data
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_NextData = GPS_NEXTDATA_NUMSAT;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_NUMSAT:
						{
							temp = 1;
							GPS_SATSTATUS_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_SATSTATUS_T = GPS_SATSTATUS_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							// Go to next data
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_NextData = GPS_NEXTDATA_HDOP;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_M1:
						{
							// Go to next data
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_NextData = GPS_NEXTDATA_GG;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_DATE:
						{
							// Store received date
							// Store day
							GPS_DAY_T = ((GPS_Digits[0] - 48) * 10) + (GPS_Digits[1] - 48);
							// Store month
							GPS_MONTH_T = ((GPS_Digits[2] - 48) * 10) + (GPS_Digits[3] - 48);
							// Store year
							GPS_YEAR_T = ((GPS_Digits[4] - 48) * 10) + (GPS_Digits[5] - 48);
							// Go to next data
							GPS_ProcesState = GPS_WAITFOREND;
							GPS_NextData = GPS_NEXTDATA_VOID;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_TIME:
						{
							// Store received time
							// Store hours
							GPS_HOURS_T = 0;
							// Store minutes
							GPS_MINUTES_T = 0;
							// Store seconds
							GPS_SECONDS_T = 0;
							// Go to next data
							if(GPSFLAG_GGA)
							{
								GPS_NextData = GPS_NEXTDATA_LATITUDE;
							}
							else if(GPSFLAG_RMC)
							{
								GPS_NextData = GPS_NEXTDATA_STATUS;
							}
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_LATITUDE:
						{
							// Store received latitude
							// Latitude =0 - 90 = 0 - 5.400 minutes
							GPS_LATITUDE_T = 0;
							GPS_LATITUDE_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_NS;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_LONGITUDE:
						{
							// Store received longitude
							// Latitude =0 - 180 = 0 - 10.800 minutes
							GPS_LONGITUDE_T = 0;
							GPS_LONGITUDE_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_EW;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_SPEED:
						{
							GPS_SPEED_T = 0;
							GPS_SPEED_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_HEADING;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_HEADING:
						{
							GPS_TRACKANGLE_T = 0;
							GPS_TRACKANGLE_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_DATE;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_HDOP:
						{
							GPS_HDOP_T = 0;
							GPS_HDOP_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_ALTITUDE;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_ALTITUDE:
						{
							GPS_ALTITUDE_T = 0;
							GPS_ALTITUDE_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_M1;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_GG:
						{
							GPS_GG_T = 0;
							GPS_GG_FRAC_T = 0;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_VOID;
							GPS_ProcesState = GPS_WAITFOREND;
							GPS_Digits_Count = 0;
							break;
						}
					}
					GPS_Digits[0] = 0;
					GPS_Digits[1] = 0;
					GPS_Digits[2] = 0;
					GPS_Digits[3] = 0;
				}
				else if(data == '.')
				{
					// Go to receiving fraction part
					GPS_ProcesState = GPS_RECEIVE_NUMFRAC;
					GPS_DataTemp_Count = 0;
					GPS_DataTemp = 0;
				}
				else if(data == 0x0D)
				{
					// CR
					GPSFLAG_CR_RECEIVED = 1;
				}
				else if(data == 0x0A)
				{
					// LF
					if(GPSFLAG_CR_RECEIVED)
					{
						// GPS_Digits[0] = high char;
						// GPS_Digits[1] = low char;
						// Check that data is OK
						if((GPS_Digits[0] == GPS_GetChar(GPS_Checksum_Save, SET))&&(GPS_Digits[1] == GPS_GetChar(GPS_Checksum_Save, RESET)))
						{
							// Data is OK, store
							for(temp = 0; temp < 24; temp++)
							{
								MODBUSReg[temp + 2] = GPS_RECDATA[temp];
							}
							// Check if data is valid
							if((GPS_VALID & _BIT15) != 0)
							{
								// Store to GPS_Data as floats
								ahrs_data.GPSData.dataValid = INVALID;
								ahrs_data.GPSData.altitude = intToFloat(GPS_ALTITUDE, GPS_ALTITUDE_FRAC);
								// Latitude, longitude in rad = res * (pi/180*180)
								ahrs_data.GPSData.latitude = intToFloat(GPS_LATITUDE, GPS_LATITUDE_FRAC);
								ahrs_data.GPSData.latitude = (ahrs_data.GPSData.latitude * PI)/32400;

								ahrs_data.GPSData.longitude = intToFloat(GPS_LONGITUDE, GPS_LONGITUDE_FRAC);
								ahrs_data.GPSData.longitude = (ahrs_data.GPSData.longitude * PI)/10800;

								ahrs_data.GPSData.speed = intToFloat(GPS_SPEED, GPS_SPEED_FRAC);
								ahrs_data.GPSData.trackAngle = intToFloat(GPS_TRACKANGLE, GPS_TRACKANGLE_FRAC);
								ahrs_data.GPSData.dataTime = ahrs_data.GPSData.dataStartTime;
								ahrs_data.GPSData.dataValid = VALID;
								// Update rotation matrix
								ahrs_updateGPSToGyro();
							}
							// Mark GPS OK
							SCR2 = SCR2 | SCR2_GPSOK;
						}
						else
						{
							// Mark GPS NOT OK
							SCR2 = SCR2 & ~SCR2_GPSOK;
						}
						GPS_NextData = GPS_NEXTDATA_VOID;
						GPS_ProcesState = GPS_WAITINGSTART;
						GPS_Digits_Count = 0;
					}
				}
				else if(data == '$')
				{
					GPS_ProcesState = GPS_RECEIVE_NUMBER;
					GPS_NextData = GPS_NEXTDATA_DATACODE;
					// Go to next data
					GPS_Digits[0] = 0;
					GPS_Digits[1] = 0;
					GPS_Digits_Count = 0;
					GPSFLAG_CR_RECEIVED = 0;
					// Mark new data time
					ahrs_data.GPSData.dataStartTime = systemTime;
					// Store rotation matrix
					ahrs_data.GPSReference = ahrs_data.rotationMatrix;
				}
				else
				{
					GPS_Digits[GPS_Digits_Count] = data;
					GPS_Digits_Count++;
				}
				break;
			}
			case GPS_RECEIVE_NUMFRAC:
			{
				if(data == ',')
				{
					// Store data and go to next data
					switch(GPS_NextData)
					{
						// Case data received is time
						case GPS_NEXTDATA_TIME:
						{
							// Store received time
							// Store hours
							GPS_HOURS_T = ((GPS_Digits[0] - 48) * 10) + (GPS_Digits[1] - 48);
							// Store minutes
							GPS_MINUTES_T = ((GPS_Digits[2] - 48) * 10) + (GPS_Digits[3] - 48);
							// Store seconds
							GPS_SECONDS_T = ((GPS_Digits[4] - 48) * 10) + (GPS_Digits[5] - 48);
							// Store milliseconds
							GPS_MILISECONDS_T = GPS_DataTemp;
							// Go to next data
							if(GPSFLAG_GGA)
							{
								GPS_NextData = GPS_NEXTDATA_LATITUDE;
							}
							else if(GPSFLAG_RMC)
							{
								GPS_NextData = GPS_NEXTDATA_STATUS;
							}
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_LATITUDE:
						{
							// Store received latitude
							// Latitude =0 - 90 = 0 - 5.400 minutes
							GPS_LATITUDE_T = ((GPS_Digits[0] - 48) * 10) + (GPS_Digits[1] - 48);
							GPS_LATITUDE_T = GPS_LATITUDE_T * 60;
							GPS_LATITUDE_T = GPS_LATITUDE_T + (((GPS_Digits[2] - 48) * 10) + (GPS_Digits[3] - 48));
							GPS_LATITUDE_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_NS;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_LONGITUDE:
						{
							// Store received longitude
							// Latitude =0 - 180 = 0 - 10.800 minutes
							GPS_LONGITUDE_T = ((GPS_Digits[0] - 48) * 100) + ((GPS_Digits[1] - 48) * 10) + (GPS_Digits[2] - 48);
							GPS_LONGITUDE_T = GPS_LONGITUDE_T * 180;
							GPS_LONGITUDE_T = GPS_LONGITUDE_T + (((GPS_Digits[3] - 48) * 10) + (GPS_Digits[4] - 48));
							GPS_LONGITUDE_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_EW;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_SPEED:
						{
							temp = 1;
							GPS_SPEED_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_SPEED_T = GPS_SPEED_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							GPS_SPEED_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_HEADING;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_HEADING:
						{
							temp = 1;
							GPS_TRACKANGLE_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_TRACKANGLE_T = GPS_TRACKANGLE_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							GPS_TRACKANGLE_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_DATE;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_HDOP:
						{
							temp = 1;
							GPS_HDOP_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_HDOP_T = GPS_HDOP_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							// Limit GPS_DataTemp to 1 decimal place
							GPS_HDOP_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_ALTITUDE;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_ALTITUDE:
						{
							temp = 1;
							GPS_ALTITUDE_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_ALTITUDE_T = GPS_ALTITUDE_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							GPS_ALTITUDE_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_M1;
							GPS_ProcesState = GPS_RECEIVE_NUMBER;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_GG:
						{
							temp = 1;
							GPS_GG_T = 0;
							do
							{
								GPS_Digits_Count--;
								GPS_GG_T = GPS_GG_T + (GPS_Digits[GPS_Digits_Count] - 48) * temp;
								temp = temp * 10;
							}
							while(GPS_Digits_Count > 0);
							GPS_GG_FRAC_T = GPS_DataTemp;
							// Go to next data
							GPS_NextData = GPS_NEXTDATA_VOID;
							GPS_ProcesState = GPS_WAITFOREND;
							GPS_Digits_Count = 0;
							break;
						}
						case GPS_NEXTDATA_DATACODE:
						case GPS_NEXTDATA_STATUS:
						case GPS_NEXTDATA_NS:
						case GPS_NEXTDATA_EW:
						case GPS_NEXTDATA_FIXVALID:
						case GPS_NEXTDATA_NUMSAT:
						case GPS_NEXTDATA_M1:
						case GPS_NEXTDATA_DATE:
						{
							// Reset
							GPS_ProcesState = GPS_WAITINGSTART;
							GPS_NextData = GPS_NEXTDATA_VOID;
							GPS_Digits_Count = 0;
							break;
						}
					}
					GPS_Digits[0] = 0;
					GPS_Digits[1] = 0;
					GPS_Digits[2] = 0;
					GPS_Digits[3] = 0;
				}
				else if(data == '$')
				{
					GPS_ProcesState = GPS_RECEIVE_NUMBER;
					GPS_NextData = GPS_NEXTDATA_DATACODE;
					// Go to next data
					GPS_Digits[0] = 0;
					GPS_Digits[1] = 0;
					GPS_Digits_Count = 0;
					GPSFLAG_CR_RECEIVED = 0;
					// Mark new data time
					ahrs_data.GPSData.dataStartTime = systemTime;
					// Store rotation matrix
					ahrs_data.GPSReference = ahrs_data.rotationMatrix;
				}
				else if(data == '*')
				{
					// Go to next data
					GPS_NextData = GPS_NEXTDATA_CHECKSUM;
					GPS_ProcesState = GPS_RECEIVE_NUMBER;
					GPS_Digits_Count = 0;
					// Save checksum
					GPS_Checksum_Save = GPS_Checksum;
				}
				else
				{
					temp = data - 48;
					GPS_DataTemp = GPS_DataTemp * 10;
					GPS_DataTemp += temp;
					GPS_DataTemp_Count++;
				}
				break;
			}
			case GPS_WAITFOREND:
			{
				if(data == '*')
				{
					// Go to next data
					GPS_NextData = GPS_NEXTDATA_CHECKSUM;
					GPS_ProcesState = GPS_RECEIVE_NUMBER;
					GPS_Digits_Count = 0;
					// Save checksum
					GPS_Checksum_Save = GPS_Checksum;
				}
				break;
			}
			default:
			{
				GPS_ProcesState = GPS_WAITINGSTART;
				break;
			}
		}
	}
	// Calculate checksum
	if(GPSFLAG_CHECKSUM_RESET)
	{
		GPSFLAG_CHECKSUM_RESET = 0;
		GPS_Checksum = data;
	}
	GPS_Checksum = GPS_Checksum^data;
}

