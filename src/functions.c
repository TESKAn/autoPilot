/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"
#include <string.h>
#include <stdlib.h>
#include "sensors/altimeter.h"

void updateExportVars(void)
{
	exportVars[0] = (int)systemTime;
	exportVars[1] = (int)ACC_X;
	exportVars[2] = (int)ACC_Y;
	exportVars[3] = (int)ACC_Z;
	exportVars[4] = (int)MAG_X;
	exportVars[5] = (int)MAG_Y;
	exportVars[6] = (int)MAG_Z;
	exportVars[7] = (int)GYRO_X;
	exportVars[8] = (int)GYRO_Y;
	exportVars[9] = (int)GYRO_Z;
	exportVars[10] = (int)(ahrs_data.Q.w * 1000000);
	exportVars[11] = (int)(ahrs_data.Q.x * 1000000);
	exportVars[12] = (int)(ahrs_data.Q.y * 1000000);
	exportVars[13] = (int)(ahrs_data.Q.z * 1000000);
	exportVars[14] = (int)(ahrs_data.AccVector.vector.pData[VECT_X] * 1000000);
	exportVars[15] = (int)(ahrs_data.AccVector.vector.pData[VECT_Y] * 1000000);
	exportVars[16] = (int)(ahrs_data.AccVector.vector.pData[VECT_Z] * 1000000);

	exportVars[17] = (int)(ahrs_data.MagVector.vector.pData[VECT_X] * 1000000);
	exportVars[18] = (int)(ahrs_data.MagVector.vector.pData[VECT_Y] * 1000000);
	exportVars[19] = (int)(ahrs_data.MagVector.vector.pData[VECT_Z] * 1000000);

	exportVars[20] = (int)(ahrs_data.GyroVector.vector.pData[VECT_X] * 1000000);
	exportVars[21] = (int)(ahrs_data.GyroVector.vector.pData[VECT_Y] * 1000000);
	exportVars[22] = (int)(ahrs_data.GyroVector.vector.pData[VECT_Z] * 1000000);

	exportVars[23] = (int)(ahrs_data.GravityVector.vector.pData[VECT_X] * 1000000);
	exportVars[24] = (int)(ahrs_data.GravityVector.vector.pData[VECT_Y] * 1000000);
	exportVars[25] = (int)(ahrs_data.GravityVector.vector.pData[VECT_Z] * 1000000);

	exportVars[26] = (int)(ahrs_data.RollPitchCorrection.vector.pData[VECT_X] * 1000000);
	exportVars[27] = (int)(ahrs_data.RollPitchCorrection.vector.pData[VECT_Y] * 1000000);
	exportVars[28] = (int)(ahrs_data.RollPitchCorrection.vector.pData[VECT_Z] * 1000000);

	exportVars[29] = (int)(ahrs_data.YawCorrection.vector.pData[VECT_X] * 1000000);
	exportVars[30] = (int)(ahrs_data.YawCorrection.vector.pData[VECT_Y] * 1000000);
	exportVars[31] = (int)(ahrs_data.YawCorrection.vector.pData[VECT_Z] * 1000000);

	exportVars[32] = (int)(ahrs_data.totalCorrectionError.vector.pData[VECT_X] * 1000000);
	exportVars[33] = (int)(ahrs_data.totalCorrectionError.vector.pData[VECT_Y] * 1000000);
	exportVars[34] = (int)(ahrs_data.totalCorrectionError.vector.pData[VECT_Z] * 1000000);

	exportVars[35] = (int)(ahrs_data.PIData.Ix * 1000000);
	exportVars[36] = (int)(ahrs_data.PIData.Iy * 1000000);
	exportVars[37] = (int)(ahrs_data.PIData.Iz * 1000000);

	exportVars[38] = (int)(ahrs_data.PIData.Rx * 1000000);
	exportVars[39] = (int)(ahrs_data.PIData.Ry * 1000000);
	exportVars[40] = (int)(ahrs_data.PIData.Rz * 1000000);

	exportVars[41] = (int)(ahrs_data.rotationMatrix.vector.pData[Rxx] * 1000000);
	exportVars[42] = (int)(ahrs_data.rotationMatrix.vector.pData[Ryx] * 1000000);
	exportVars[43] = (int)(ahrs_data.rotationMatrix.vector.pData[Rzx] * 1000000);

	exportVars[44] = (int)(ahrs_data.rotationMatrix.vector.pData[Rxy] * 1000000);
	exportVars[45] = (int)(ahrs_data.rotationMatrix.vector.pData[Ryy] * 1000000);
	exportVars[46] = (int)(ahrs_data.rotationMatrix.vector.pData[Rzy] * 1000000);

	exportVars[47] = (int)(ahrs_data.rotationMatrix.vector.pData[Rxz] * 1000000);
	exportVars[48] = (int)(ahrs_data.rotationMatrix.vector.pData[Ryz] * 1000000);
	exportVars[49] = (int)(ahrs_data.rotationMatrix.vector.pData[Rzz] * 1000000);


}

uint32_t getSystemTime(void)
{
	uint32_t time = 0;
	uint32_t timeFrac = 0;
	time = systemTime * 100;	// 1 time tick is 10 usec
	timeFrac = (uint32_t)(TIM14->CNT);
	timeFrac = timeFrac / 10;	// Period is 1000 usec
	time = time + timeFrac;
	return time;
}


float32_t getFTime(void)
{
	float32_t time = 0;
	float32_t timeFrac = 0;
	time = systemTime;
	timeFrac = (float32_t)(TIM14->CNT);
	timeFrac = timeFrac / 1000;
	time = time + timeFrac;
	return time;
}

ErrorStatus FS_Initialize(void)
{
	if(!SD_INITIALIZED)
	{
		// Try to initialize SD card
		if(disk_initialize(0) == RES_OK)
		{
			SD_INITIALIZED = 1;
			return SUCCESS;
		}
	}
	else
	{
		return SUCCESS;
	}
	return ERROR;
}

void storeAHRSAngles(void)
{
	float32_t temp = 0;
	int16_t temp1 = 0;

	// Store angles
	temp = -asinf(ahrs_data.rotationMatrix.vector.pData[Rzx]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_PITCH = (uint16_t) temp1;

	temp = atan2f( ahrs_data.rotationMatrix.vector.pData[Rzy],  ahrs_data.rotationMatrix.vector.pData[Rzz]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_ROLL = (uint16_t) temp1;

	temp = atan2f( ahrs_data.rotationMatrix.vector.pData[Ryx],  ahrs_data.rotationMatrix.vector.pData[Rxx]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_YAW = (uint16_t) temp1;
}

void openLog(void)
{
	unsigned int bytesWritten;

	// Check that SD card is mounted
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
		#ifdef DEBUG_USB
			sendUSBMessage("SD card mounted");
		#endif
		}
		else
		{
		#ifdef DEBUG_USB
			sendUSBMessage("SD card not mounted!");
		#endif
			return;
		}
	}

	// Generate file name
	// File name = "/LOG_ddmmyyyy_hhmmss.txt"

	// Fill buffer
	FSBuffer[0] = '/';
	FSBuffer[1] = 'L';
	FSBuffer[2] = 'O';
	FSBuffer[3] = 'G';
	FSBuffer[4] = '_';
	FSBuffer[5] = charFromNumber(GPS_DAY / 10);
	FSBuffer[6] = charFromNumber(GPS_DAY % 10);
	FSBuffer[7] = charFromNumber(GPS_MONTH / 10);
	FSBuffer[8] = charFromNumber(GPS_MONTH % 10);
	FSBuffer[9] = '2';
	FSBuffer[10] = '0';
	FSBuffer[11] = '1';
	FSBuffer[12] = '2';
	FSBuffer[13] = '_';
	FSBuffer[14] = charFromNumber(GPS_HOURS / 10);
	FSBuffer[15] = charFromNumber(GPS_HOURS % 10);
	FSBuffer[16] = charFromNumber(GPS_MINUTES / 10);
	FSBuffer[17] = charFromNumber(GPS_MINUTES % 10);
	FSBuffer[18] = charFromNumber(GPS_SECONDS / 10);
	FSBuffer[19] = charFromNumber(GPS_SECONDS % 10);
	FSBuffer[20] = '.';
	FSBuffer[21] = 'c';
	FSBuffer[22] = 's';
	FSBuffer[23] = 'v';
	FSBuffer[24] = 0;
	// Open file
	if(f_open(&logFile, FSBuffer, FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		//flag error
	#ifdef DEBUG_USB
		sendUSBMessage("Open file error");
	#endif
		return;
	}

	// Write first line
	f_write(&logFile, "Time;GPS Lock;Lat;;Lon;;Alt;Speed;Track angle;HDOP;GG;AccX;AccY;AccZ;GyroX;GyroY;GyroZ;MagX;MagY;MagZ;Baro;AirSpeed\r\n", 136, &bytesWritten);
#ifdef DEBUG_USB
	sendUSBMessage("Log opened");
#endif
}

void closeLog(void)
{
	// Flush buffers to SD
	uint32_t BufferCount = SD_Buf1Count;
	unsigned int temp = 0;
	// Make pointer to buffer
	char* Buffer = &SD_Buffer1[0];
	// If we are using buffer 2, change pointer
	if(SD_BUF_IN_USE)
	{
		Buffer = &SD_Buffer2[0];
		BufferCount = SD_Buf2Count;
	}
	f_write(&logFile, Buffer, BufferCount, &temp);

	// Close.
	SCR2 = SCR2 & ~SCR2_LOGOPEN;
	SD_WRITE_LOG = 0;
	f_close(&logFile);
#ifdef DEBUG_USB
	sendUSBMessage("Log closed");
#endif
}

void write_toLog(void)
{
	//uint16_t temp1, temp2;
	//uint32_t temp = 0;
	// Make pointer to buffer place
	uint32_t* BufferPointer = &SD_Buf1Count;
	// Make pointer to buffer
	char* Buffer = &SD_Buffer1[0];
	// If we are using buffer 2, change pointers
	if(SD_BUF_IN_USE)
	{
		Buffer = &SD_Buffer2[0];
		BufferPointer = &SD_Buf2Count;
		// Check that we are not writing buffer 2
		if(SD_WRITING_BUF2) return;
	}
	else
	{
		// Check that we are not writing buffer 1
		if(SD_WRITING_BUF1) return;
	}
	// Store time
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d:%d:%d;", GPS_HOURS, GPS_MINUTES, GPS_SECONDS);
	// Store GPS data
	// GPS lock
	if ((GPS_VALID & 32768) != 0)
	{
		Buffer[*BufferPointer] = '1';
	}
	else
	{
		Buffer[*BufferPointer] = '0';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Latitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_LATITUDE, GPS_LATITUDE_FRAC);
	// N/S
	if((GPS_NS_EW & 1) != 0)
	{
		Buffer[*BufferPointer] = 'S';
	}
	else
	{
		Buffer[*BufferPointer] = 'N';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Longitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_LONGITUDE, GPS_LONGITUDE_FRAC);
	// E/W
	if((GPS_NS_EW & 2) != 0)
	{
		Buffer[*BufferPointer] = 'W';
	}
	else
	{
		Buffer[*BufferPointer] = 'E';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Altitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_ALTITUDE, GPS_ALTITUDE_FRAC);
	// Speed
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_SPEED, GPS_SPEED_FRAC);
	// Track angle
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_TRACKANGLE, GPS_TRACKANGLE_FRAC);
	// HDOP
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_HDOP, GPS_HDOP_FRAC);
	// GG
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_GG, GPS_GG_FRAC);

	// Store accelerometer X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)ACC_X, (int16_t)ACC_Y, (int16_t)ACC_Z);
	// Store gyro X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)GYRO_X, (int16_t)GYRO_Y, (int16_t)GYRO_Z);
	// Store magnetometer X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)MAG_X, (int16_t)MAG_Y, (int16_t)MAG_Z);
	/*
	// Store barometer pressure
	temp1 = (uint16_t)(fusionData._altimeter.pressure / 1000);
	temp2 = (uint16_t)(fusionData._altimeter.pressure % 1000);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d%d", temp1, temp2);

	temp1 = (uint16_t)(fusionData._altimeter.pressure_frac);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], ",%d;", temp1);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;", AIN3);
*/
/*
	// Store power Uin, Iin, mAh used, T1, T2, T3 - always positive 16 bit
	*BufferPointer = *BufferPointer + storeNumber(VOLTAGE, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(CURRENT, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(MAH, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T1, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T2, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T3, Buffer, *BufferPointer);
*/

	// Write \r\n
	Buffer[*BufferPointer] = 0x0d;
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = 0x0a;
	*BufferPointer = *BufferPointer + 1;
	// Check value
	if(*BufferPointer > SD_BUF_MESSAGE_LIMIT)
	{
		if(!SD_BUF_IN_USE)
		{
			// Was filling buffer 1, write it and move to buffer 2
			SD_WRITING_BUF1 = 1;
			SD_BUF_IN_USE = 1;
			Buffer[*BufferPointer] = 0;
		}
		else
		{
			// Was filling buffer 2, write it and move to buffer 1
			SD_WRITING_BUF2 = 1;
			SD_BUF_IN_USE = 0;
			Buffer[*BufferPointer] = 0;
		}

	}
}

int storeNumber(uint16_t number, char* buffer, int offset)
{
	// Stores 16 bit number to buffer, starting at offset, ending with ;
	char temp = 0;
	int charWritten = offset;
	if(number > 10000)
	{
		temp = charFromNumber(number / 10000);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 1000)
	{
		temp = charFromNumber(number / 1000);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 100)
	{
		temp = charFromNumber((number / 100) % 10);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 10)
	{
		temp = charFromNumber((number / 10) % 10);
		buffer[offset] = temp;
		offset++;
	}
	temp = charFromNumber(number % 10);
	buffer[offset] = temp;
	offset++;
	buffer[offset] = ';';
	offset++;
	return offset - charWritten;
}
int storeNegativeNumber(uint16_t number, char* buffer, int offset)
{
	// Stores 16 bit number to buffer, starting at offset, ending with ;
	char temp = 0;
	int charWritten = offset;
	int16_t negNumber = (int16_t)number;
	if(negNumber < 0)
	{
		// Write -
		buffer[offset] = '-';
		offset++;
		// Make positive
		negNumber = negNumber * -1;
	}
	if(negNumber > 10000)
	{
		temp = charFromNumber(negNumber / 10000);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 1000)
	{
		temp = charFromNumber(negNumber / 1000);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 100)
	{
		temp = charFromNumber((negNumber / 100) % 10);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 10)
	{
		temp = charFromNumber((negNumber / 10) % 10);
		buffer[offset] = temp;
		offset++;
	}
	temp = charFromNumber(negNumber % 10);
	buffer[offset] = temp;
	offset++;
	buffer[offset] = ';';
	offset++;
	return offset - charWritten;
}

//
ErrorStatus int16ToStr(int16_t value, char* text, char* str)
{
	int n = strlen(text);
	strcpy (str, text);
	n += sprintf (str+n, "%d", value);
	return SUCCESS;
}

ErrorStatus uint32ToStr(uint32_t value, char* text, char* str)
{
	//int n = strlen(text);
	strcpy (str, text);
	//n += sprintf (str+n, "%d", value);
	return SUCCESS;
}

// value - floating point value to convert
// text - text to append before value
// str - buffer for converted string

ErrorStatus float32ToStr(float32_t value, char* text, char* str)
{

	int n = strlen(text);
	int i = 0;
	float multi = 0;
	int num = 0;
	int exp = 0;
	strcpy (str, text);

	if(value != 0)
	{
		// Store +/-
		if(value < 0)
		{
			// Negative number
			// Store - sign
			strcat(str, "-");
			n++;
			// Make positive
			value *= -1;
		}
		// Move value up or down
		exp = 0;
		// If larger or equal to 10, divide by 10
		while(value >= 10.0f)
		{
			value /= 10;
			exp = exp + 1;
		}
		// If smaller than 1, multiply by 10
		while(value < 1)
		{
			value *= 10;
			exp = exp - 1;
		}

		num = (int)value;
		n += sprintf (str+n, "%d", num);
		strcat(str, ".");
		n++;
		value = value - (float)num;
		multi = 1000000000;
		for(i=0; i < 9; i++)
		{
			value = value * 10;
			multi = multi / 10;
			num = (int)value;
			if(num == 0)
			{
				strcat(str, "0");
				n++;
			}
			else
			{
				value = value * multi;
				num = (int)value;
				n += sprintf (str+n, "%d", num);
				break;
			}
		}
		// Store e
		strcat(str, "e");
		n++;
		// Store exp
		n += sprintf (str+n, "%d", exp);
		}
	else
	{
		sprintf (str+n, "%d", 0);
	}
	return SUCCESS;
}

ErrorStatus strToFloat32(float32_t* result, char* file, char* str)
{
	char* strBeginning = 0;
	char* strEnd = 0;
	char* numBeginning = 0;
	uint8_t strBeginLen = strlen(str);
	float32_t convertedValue = 0;

	// file - file that contains data
	// str - string that marks data, in form of val1=1234;

	strBeginning = strstr(file, str);
	// Check that it is not null
	if(strBeginning != NULL)
	{
		// Get pointer to where in file string ends
		strEnd = strstr(strBeginning, ";");

		numBeginning = strBeginning + strBeginLen;

		// Check that we have ending
		if(strEnd == NULL)
		{
			// Return error
			return ERROR;
		}
		// Convert
		convertedValue = strtof(numBeginning, (char**)strEnd);
	}
	else
	{
		return ERROR;
	}
	*result = convertedValue;
	return SUCCESS;
}

uint16_t strTouint16(char* file, char* str)
{
	char* strBeginning = 0;
	char* strEnd = 0;
	uint8_t convert = 0;
	uint8_t strBeginLen = strlen(str);
	uint16_t convertedValue = 0;
	uint8_t i = 0;
	//uint32_t value = 0;
	char currentChar = 0;
	// file - file that contains data
	// str - string that marks data, in form of val1=1234;
	strBeginning = strstr(file, str);
	// Check that it is not null
	if(strBeginning != NULL)
	{
		// Get pointer to where in file string ends
		strEnd = strstr(strBeginning, ";");
		// Count how many chars to convert
		convert = (uint8_t)((uint32_t)(strEnd - strBeginning) - strBeginLen);
		// Convert
		for(i = 0; i < convert; i++)
		{
			convertedValue = convertedValue * 10;
			// Get value
			currentChar = strBeginning[5 + i];
			convertedValue = convertedValue + (uint16_t)numberFromChar(currentChar);
		}
	}
	else
	{
		convertedValue = 0;
	}
	return convertedValue;
}



char charFromNumber(uint8_t number)
{
	return number + 48;
}
uint8_t numberFromChar(char c)
{
	return c - 48;
}

// Function to mount SD card
ErrorStatus mountSDCard(void)
{
	// Check if card is mounted
	if(SD_MOUNTED)
	{
		// If it is, return
		#ifdef DEBUG_USB
			sendUSBMessage("FS already mounted");
		#endif
		return ERROR;
	}
	// Load drive
	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("FS mount error");
		#endif
		// Flag error
		f_mount(0,0);
		return ERROR;
	}
	driveStatus = disk_initialize(0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT))
	{
		#ifdef DEBUG_USB
			sendUSBMessage("Drive Status error");
		#endif
		// Flag error.
		f_mount(0,0);
		return ERROR;
	}
	// Mark SD card is mounted
	SD_MOUNTED = 1;
	return SUCCESS;
}

// Function to unmount SD card
ErrorStatus unmountSDCard(void)
{
	if(SD_MOUNTED)
	{
		// Close all open files
		f_mount(0,0);
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

ErrorStatus loadSingleSetting(char* name, float32_t* storeLocation)
{
	FIL settingsFile;
	unsigned int bytesToRead = 0;
	unsigned int readBytes = 0;
	unsigned int bytesProcessed = 0;
	unsigned int fileSize = 0;

	// Check that SD card is mounted
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}

	fileSize = f_size(&settingsFile);

	do
	{
		bytesToRead = 255;
		if((bytesToRead + bytesProcessed) > fileSize)
		{
			bytesToRead = fileSize - bytesProcessed;
		}
		// Read file to buffer
		if(f_read (&settingsFile, &FSBuffer[0], bytesToRead, &readBytes) != FR_OK)
		{
	#ifdef DEBUG_USB
			sendUSBMessage("File read error");
	#endif
			// Close and unmount.
			f_close(&settingsFile);
			return ERROR;
		}
		// Store null character to last place
		FSBuffer[254] = '\0';
		// Check if required data is in settings file
		if(strToFloat32(storeLocation, &FSBuffer[0], name) == SUCCESS)
		{
			// Return success
			//Close and unmount.
			f_close(&settingsFile);
			return SUCCESS;
		}
		else
		{
			// Set processed bytes
			bytesProcessed = bytesProcessed + 200;
			// Else seek for next section
			f_lseek(&settingsFile, bytesProcessed);
		}
	}
	while(bytesProcessed < fileSize);

	// Close
	f_close(&settingsFile);

	return ERROR;
}

// Store parameters calculated in software
// Like PID coefficients etc.
ErrorStatus storeRunningValues(void)
{
	FIL settingsFile;
#ifdef DEBUG_USB
	sendUSBMessage("Begin saving parameters");
#endif
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings1.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}
	/*
	// Store number
	float32ToStr(ahrs_data.accRate, "accRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);
	*/
	// Close file
	f_close(&settingsFile);

#ifdef DEBUG_USB
	sendUSBMessage("Parameters saved");
#endif

	return SUCCESS;
}

// Store program parameters
ErrorStatus storeSettings(void)
{
	FIL settingsFile;
#ifdef DEBUG_USB
	sendUSBMessage("Begin saving settings");
#endif
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings1.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}
	// Store settings
	// Acc rate
	// Store number
	float32ToStr(ahrs_data.accRate, "accRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Acceleration rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Gyro rate
	// Store number
	float32ToStr(ahrs_data.gyroRate, "gyroRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Gyro rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag rate
	// Store number
	float32ToStr(ahrs_data.magRate, "magRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetometer rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag inclination
	// Store number
	float32ToStr(DEFAULT_MAG_INCLINATION, "magInclination=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetic field inclination\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag declination
	// Store number
	float32ToStr(DEFAULT_MAG_DECLINATION, "magDeclination=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetic field declination\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// PID threshold
	// Store number
	float32ToStr(ahrs_data.PIDErrorThreshold, "PIDThreshold=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimal PID error to use\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Minimal rotation rate that is detected
	// Store number
	float32ToStr(ahrs_data.MinRotationRate, "MinRotationRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimal detectable rotation rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Soft mag matrix
	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxx], "softMagRxx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Softmag matrix coefficients\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryx], "softMagRyx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzx], "softMagRzx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxy], "softMagRxy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryy], "softMagRyy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzy], "softMagRzy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxz], "softMagRxz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryz], "softMagRyz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzz], "softMagRzz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Hard mag vector
	// Store number
	float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_X], "hardMagX=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Hard mag vector\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_Y], "hardMagY=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_Z], "hardMagZ=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Hard mag scale
	// Store number
	float32ToStr(SOFTMAG_SCALE, "hardMagScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Hard mag scale\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// PID parameters
	// Store number
	float32ToStr(ahrs_data.PIData.Kix, "Kix=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("PID parameters\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.Kpx, "Kpx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.eMax, "PID_MaxErr=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.eMin, "PID_MinErr=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.maxIx, "PID_MaxI=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.minIx, "PID_MinI=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.rMax, "PID_MaxR=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.PIData.rMin, "PID_MinR=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);


	// Roll, pitch, yaw correction factors
	// Store number
	float32ToStr(ahrs_data.RollPitchCorrectionScale, "rollPitchCorrectionScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Roll, pitch, yaw correction factors\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	float32ToStr(ahrs_data.YawCorrectionScale, "yawCorrectionScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Samples to discard on startup
	// Store number
	float32ToStr(ahrs_data.sampleDiscardCount, "discardCount=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Samples to discard on startup\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Minimum GPS speed to use GPS for yaw
	// Store number
	float32ToStr(ahrs_data.MinGPSSpeed, "useGPSSpeed=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimum GPS speed to use GPS for yaw\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Close file
	f_close(&settingsFile);

#ifdef DEBUG_USB
	sendUSBMessage("Settings saved");
#endif
	return SUCCESS;
}

ErrorStatus loadSettings(void)
{
	// Load all settings, one at a time
	// Acceleration rate
	if(loadSingleSetting("accRate=", &(ahrs_data.accRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.accRate = DEFAULT_ACC_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.accRate, "accRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Gyroscope rate
	if(loadSingleSetting("gyroRate=", &(ahrs_data.gyroRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.gyroRate = DEFAULT_GYRO_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.gyroRate, "gyroRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Magnetometer rate
	if(loadSingleSetting("magRate=", &(ahrs_data.magRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.magRate = DEFAULT_MAG_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.magRate, "magRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Ki factor
	if(loadSingleSetting("Kix=", &(ahrs_data.PIData.Kix)) != SUCCESS)
	{
		// Load default value
		ahrs_data.PIData.Kix = DEFAULT_KI;
	}
	// Store
	ahrs_data.PIData.Kiy = ahrs_data.PIData.Kix;
	ahrs_data.PIData.Kiz = ahrs_data.PIData.Kix;
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.PIData.Kix, "Kix=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif

	// Kp factor
	if(loadSingleSetting("Kpx=", &(ahrs_data.PIData.Kpx)) != SUCCESS)
	{
		// Load default value
		ahrs_data.PIData.Kpx = DEFAULT_KP;
	}
	// Store
	ahrs_data.PIData.Kpy = ahrs_data.PIData.Kpx;
	ahrs_data.PIData.Kpz = ahrs_data.PIData.Kpx;
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.PIData.Kpx, "Kpx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Load mag matrix
	// Rxx
	if(loadSingleSetting("softMagRxx=", &(ahrs_data.rotationMatrix.vector.pData[Rxx])) != SUCCESS)
	{
		// Load default value
		ahrs_data.rotationMatrix.vector.pData[Rxx] = SOFTMAG_DEFAULT_RXX;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.rotationMatrix.vector.pData[Rxx], "softMagRxx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif

	// Ryx
	if(loadSingleSetting("softMagRyx=", &(ahrs_data.rotationMatrix.vector.pData[Ryx])) != SUCCESS)
	{
		// Load default value
		ahrs_data.rotationMatrix.vector.pData[Ryx] = SOFTMAG_DEFAULT_RYX;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.rotationMatrix.vector.pData[Ryx], "softMagRyx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif



#ifdef DEBUG_USB
	sendUSBMessage("Settings loaded successfully");
#endif
	return SUCCESS;
}


void extPeripheralInit(void)
{
	// Turn sensor power ON
	SENSOR_POWER_ON;
	// Initialize SD card
	FS_Initialize();
#ifdef DEBUG_USB
	sendUSBMessage("Power ON");
#endif
	// Init ahrs structure
	initAHRSStructure(&ahrs_data);
#ifdef DEBUG_USB
	sendUSBMessage("AHRS initialized");
#endif
#ifdef DEBUG_USB
	sendUSBMessage("Configuring GPS");
#endif
	// Setup GPS
	GPSSetDataOutput();
#ifdef DEBUG_USB
	sendUSBMessage("GPS configured");
#endif
	Delayms(100);
	// Setup sensors
#ifdef DEBUG_USB
	sendUSBMessage("Configuring sensors");
#endif
#ifdef DEBUG_USB
	sendUSBMessage("Set Power sensor");
#endif
	// set PS busy
	PSBUSY = 1;
	// Short delay
	Delaynus(5000);
	// Reset PS
	PSReset();
	Delayms(100);
	PSBUSY = 0;
#ifdef DEBUG_USB
	sendUSBMessage("Configure I2C sensors");
#endif
	sensorInit();
#ifdef DEBUG_USB
	sendUSBMessage("Enable ADC");
#endif
	ADC_ENABLED = 1;
	// Mark sensors initiated
	EXTSENS_INIT_DONE = 1;
	// Mark null sensor
	//EXTSENS_NULLING_GYRO = 1;
#ifdef DEBUG_USB
	sendUSBMessage("Sensors configured");
#endif
	Delayms(100);
#ifdef DEBUG_USB
	sendUSBMessage("Reset matrix");
#endif
	ahrs_resetRotationMatrix();
	ahrs_resetQuaternion();
#ifdef DEBUG_USB
	sendUSBMessage("AutoPilot OnLine");
	sendUSBMessage("initialization done");
#endif
}

void Delayms(uint32_t ms)
{
	uint32_t time = systemTime - 1;
	uint32_t deltaTime = 0;
	while(deltaTime < ms)
	{
		Delaynus(1500);
		if(systemTime == time)
		{
			// Error - time is not counting, break the loop
			Delaynus(ms * 1000);
			break;
		}
		deltaTime = systemTime - time;
	}
}

/*
 * Function Name  : Delaynus
 * Description    : Inserts a delay time abort nus.
 * Input          :nus
 * Output         : None
 * Return         : None
 */
void Delaynus(vu32 nus)
{
    u8 nCount;

    while (nus--)
    {
        for (nCount = 6; nCount != 0; nCount--);
    }
}

void transferDMA_USART2(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART2 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART2);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_4;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//    USART2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART2, &DMAInitStructure);

	//Enable DMA1 stream 0 - USART2 TX
	DMA_Cmd(DMA_USART2, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

void transferDMA_USART3(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART3 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART3);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;//    USART3_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART3, &DMAInitStructure);

	//Enable DMA1 stream 4 - USART3 TX
	DMA_Cmd(DMA_USART3, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART3, DMA_IT_TC, ENABLE);
}

float32_t intToFloat(int whole, int frac)
{
	float32_t result = 0;
	float32_t temp = 0;
	temp = (float32_t)frac;
	while(temp > 1.0f)
	{
		temp = temp / 10;
	}
	result = (float32_t)whole + temp;
	return result;
}

void sendUSBMessage(char* message)
{
	int len = strlen(message);
	int i = 0;
	// Check USB is connected
 	if(USB_OTG_dev.dev.device_status == USB_OTG_CONFIGURED)
 	{
		// Check length
 		if(len > 60)
 		{
 			len = 60;
 		}
		Buffer[0] = 2;
		Buffer[1] = 3;
		for(i = 0; i < len; i++)
		{
			Buffer[i + 2] =  (uint8_t)message[i];
		}
		Buffer[i + 2] = 0;
		USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64);
 	}
}
