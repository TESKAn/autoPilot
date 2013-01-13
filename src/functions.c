/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"
#include <string.h>

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
	temp = -asin(ahrs_data.rotationMatrix.vector.pData[Rzx]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_PITCH = (uint16_t) temp1;

	temp = atan2( ahrs_data.rotationMatrix.vector.pData[Rzy],  ahrs_data.rotationMatrix.vector.pData[Rzz]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_ROLL = (uint16_t) temp1;

	temp = atan2( ahrs_data.rotationMatrix.vector.pData[Ryx],  ahrs_data.rotationMatrix.vector.pData[Rxx]) * 1000;
	temp1 = (int16_t)temp;
	AHRS_YAW = (uint16_t) temp1;
}

void openLog(void)
{
	unsigned int bytesWritten;
	// Open file for writing
	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
		//flag error
#ifdef DEBUG_USB
	sendUSBMessage("FS mount error");
#endif
	}
	driveStatus = disk_status (0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT)
		   )
	{
		//flag error.
#ifdef DEBUG_USB
	sendUSBMessage("Drive status error");
#endif
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
	}

	// Write first line
	f_write(&logFile, "Time;GPS Lock;Lat;;Lon;;Alt;Speed;Track angle;HDOP;GG;AccX;AccY;AccZ;GyroX;GyroY;GyroZ;MagX;MagY;MagZ;Baro;Voltage;Current;mAh;T1;T2;T3\r\n", 136, &bytesWritten);
#ifdef DEBUG_USB
	sendUSBMessage("Log opened");
#endif
}

void closeLog(void)
{
	//Close and unmount.
	SCR2 = SCR2 & ~SCR2_LOGOPEN;
	SD_WRITE_LOG = 0;
	f_close(&logFile);
	f_mount(0,0);
#ifdef DEBUG_USB
	sendUSBMessage("Log closed");
#endif
}

void write_toLog(void)
{
	unsigned int bytesWritten;
	unsigned int uiByteCount = 0;
	//FSBuffer = malloc(128);
	// Store time
	FSBuffer[0] = charFromNumber(GPS_HOURS / 10);
	FSBuffer[1] = charFromNumber(GPS_HOURS % 10);
	FSBuffer[2] = ':';
	FSBuffer[3] = charFromNumber(GPS_MINUTES / 10);
	FSBuffer[4] = charFromNumber(GPS_MINUTES % 10);
	FSBuffer[5] = ':';
	FSBuffer[6] = charFromNumber(GPS_SECONDS / 10);
	FSBuffer[7] = charFromNumber(GPS_SECONDS % 10);
	FSBuffer[8] = ';';
	uiByteCount = 9;
	// Store GPS data
	// GPS lock
	if ((GPS_VALID & 32768) != 0)
	{
		FSBuffer[uiByteCount] = '1';
	}
	else
	{
		FSBuffer[uiByteCount] = '0';
	}
	uiByteCount++;
	FSBuffer[uiByteCount] = ';';
	uiByteCount++;
	// Latitude
	uiByteCount += storeNegativeNumber(GPS_LATITUDE, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// Latitude frac
	uiByteCount += storeNegativeNumber(GPS_LATITUDE_FRAC, FSBuffer, uiByteCount);
	// N/S
	if((GPS_NS_EW & 1) != 0)
	{
		FSBuffer[uiByteCount] = 'S';
	}
	else
	{
		FSBuffer[uiByteCount] = 'N';
	}
	uiByteCount++;
	FSBuffer[uiByteCount] = ';';
	uiByteCount++;
	// Longitude
	uiByteCount += storeNegativeNumber(GPS_LONGITUDE, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// Longitude frac
	uiByteCount += storeNegativeNumber(GPS_LONGITUDE_FRAC, FSBuffer, uiByteCount);
	// E/W
	if((GPS_NS_EW & 2) != 0)
	{
		FSBuffer[uiByteCount] = 'W';
	}
	else
	{
		FSBuffer[uiByteCount] = 'E';
	}
	uiByteCount++;
	FSBuffer[uiByteCount] = ';';
	uiByteCount++;
	// Altitude
	uiByteCount += storeNegativeNumber(GPS_ALTITUDE, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// Altitude frac
	uiByteCount += storeNegativeNumber(GPS_ALTITUDE_FRAC, FSBuffer, uiByteCount);
	// Speed
	uiByteCount += storeNegativeNumber(GPS_SPEED, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// Speed frac
	uiByteCount += storeNegativeNumber(GPS_SPEED_FRAC, FSBuffer, uiByteCount);
	// Track angle
	uiByteCount += storeNegativeNumber(GPS_TRACKANGLE, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// Track angle frac
	uiByteCount += storeNegativeNumber(GPS_TRACKANGLE_FRAC, FSBuffer, uiByteCount);
	// HDOP
	uiByteCount += storeNegativeNumber(GPS_HDOP, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// HDOP frac
	uiByteCount += storeNegativeNumber(GPS_HDOP_FRAC, FSBuffer, uiByteCount);
	// GG
	uiByteCount += storeNegativeNumber(GPS_GG, FSBuffer, uiByteCount);
	// Change ; to ,
	FSBuffer[uiByteCount - 1] = ',';
	// GG frac
	uiByteCount += storeNegativeNumber(GPS_GG_FRAC, FSBuffer, uiByteCount);

	// Store accelerometer X, Y, Z
	uiByteCount += storeNegativeNumber(ACC_X, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(ACC_Y, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(ACC_Z, FSBuffer, uiByteCount);
	// Store gyro X, Y, Z
	uiByteCount += storeNegativeNumber(GYRO_X, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(GYRO_Y, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(GYRO_Z, FSBuffer, uiByteCount);
	// Store magnetometer X, Y, Z
	uiByteCount += storeNegativeNumber(MAG_X, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(MAG_Y, FSBuffer, uiByteCount);
	uiByteCount += storeNegativeNumber(MAG_Z, FSBuffer, uiByteCount);
	// Store baro altitude
	uiByteCount += storeNegativeNumber(BARO, FSBuffer, uiByteCount);
	// Store power Uin, Iin, mAh used, T1, T2, T3 - always positive 16 bit
	uiByteCount += storeNumber(VOLTAGE, FSBuffer, uiByteCount);
	uiByteCount += storeNumber(CURRENT, FSBuffer, uiByteCount);
	uiByteCount += storeNumber(MAH, FSBuffer, uiByteCount);
	uiByteCount += storeNumber(T1, FSBuffer, uiByteCount);
	uiByteCount += storeNumber(T2, FSBuffer, uiByteCount);
	uiByteCount += storeNumber(T3, FSBuffer, uiByteCount);


	// Write \r\n
	FSBuffer[uiByteCount] = 0x0d;
	uiByteCount++;
	FSBuffer[uiByteCount] = 0x0a;
	uiByteCount++;
	f_write(&logFile, FSBuffer, uiByteCount, &bytesWritten);

	// Increase flush counter
	FatFS_FlushBuffer++;
	if(FatFS_FlushBuffer > FATFS_FLUSH_COUNT)
	{
		FatFS_FlushBuffer = 0;
		f_sync(&logFile);
	}
	// Free memory
	//free(FSBuffer);
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

ErrorStatus float32ToStr(float32_t value, char* text, char* str)
{
	int n = strlen(text);
	int i = 0;
	float multi = 0;
	int num = 0;
	strcpy (str, text);
	num = (int)value;
	n += sprintf (str+n, "%d", num);
	strcat(str, ".");
	n++;
	value = value - (float)num;
	multi = 10000000000;
	for(i=0; i < 10; i++)
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
			sprintf (str+n, "%d", num);
			break;
		}
	}
	return SUCCESS;
}

ErrorStatus strToFloat32(float32_t* result, char* file, char* str)
{
	char* strBeginning = 0;
	char* strEnd = 0;
	uint8_t convert = 0;
	uint8_t strBeginLen = strlen(str);
	float32_t convertedValue = 0;
	float32_t multiplier = 0;
	float32_t fraction = 0;
	uint8_t i = 0;
	uint8_t part = 0;
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
			currentChar = strBeginning[strBeginLen + i];
			if(part == 0)
			{
				if(currentChar == '.')
				{
					part = 1;
					multiplier = 1;
				}
				else
				{
					convertedValue = convertedValue * 10;
					// Get value
					convertedValue = convertedValue + (float32_t)numberFromChar(currentChar);
				}
			}
			else
			{
				fraction = fraction * 10;
				multiplier = multiplier * 10;
				// Get value
				fraction = fraction + (float32_t)numberFromChar(currentChar);
			}
		}
		// Merge
		fraction = fraction / multiplier;
		convertedValue = convertedValue + fraction;
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

ErrorStatus loadSettings(void)
{
	FATFS FileSystemObject;
	DSTATUS driveStatus;
	FIL settingsFile;
	unsigned int bytesToRead = 0;
	unsigned int readBytes = 0;

	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
#ifdef DEBUG_USB
	sendUSBMessage("FS mount error");
#endif
		//flag error
		f_mount(0,0);
		return ERROR;
	}
	driveStatus = disk_status (0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT)
		   )
	{
#ifdef DEBUG_USB
	sendUSBMessage("Drive Status error");
#endif
		//flag error.
		f_mount(0,0);
		return ERROR;
	}
	// Open file
	if(f_open(&settingsFile, "/settings.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
#ifdef DEBUG_USB
	sendUSBMessage("File open error");
#endif
		//flag error
		f_mount(0,0);
		return ERROR;
	}
	// Read file to buffer
	bytesToRead = f_size(&settingsFile);
	if(f_read (&settingsFile, &FSBuffer[0], bytesToRead, &readBytes) != FR_OK)
	{
#ifdef DEBUG_USB
		sendUSBMessage("File read error");
#endif
		//Close and unmount.
		f_close(&settingsFile);
		f_mount(0,0);
		return ERROR;
	}
	// Process data
	if(strToFloat32(&(ahrs_data.accRate), &FSBuffer[0], "accRate=") == SUCCESS)
	{
#ifdef DEBUG_USB

		float32ToStr(ahrs_data.accRate, "accRate=", StringBuffer);
		sendUSBMessage(StringBuffer);
#endif
	}
	else
	{
		ahrs_data.accRate = DEFAULT_ACC_RATE;

	#ifdef DEBUG_USB
		sendUSBMessage("Error reading accRate");
		sendUSBMessage("Using default values");
	#endif
	}

	if(strToFloat32(&(ahrs_data.gyroRate), &FSBuffer[0], "gyroRate=") == SUCCESS)
	{
#ifdef DEBUG_USB
		float32ToStr(ahrs_data.gyroRate, "gyroRate=", StringBuffer);
		sendUSBMessage(StringBuffer);
#endif
	}
	else
	{
		ahrs_data.gyroRate = DEFAULT_ACC_RATE;

	#ifdef DEBUG_USB
		sendUSBMessage("Error reading gyroRate");
		sendUSBMessage("Using default values");
	#endif
	}

	if(strToFloat32(&(ahrs_data.magRate), &FSBuffer[0], "magRate=") == SUCCESS)
	{
#ifdef DEBUG_USB
		float32ToStr(ahrs_data.magRate, "magRate=", StringBuffer);
		sendUSBMessage(StringBuffer);
#endif
	}
	else
	{
		ahrs_data.magRate = DEFAULT_ACC_RATE;

	#ifdef DEBUG_USB
		sendUSBMessage("Error reading magRate");
		sendUSBMessage("Using default values");
	#endif
	}

	if(strToFloat32(&(ahrs_data.PIData.Kix), &FSBuffer[0], "Kix=") == SUCCESS)
	{
		ahrs_data.PIData.Kiy = ahrs_data.PIData.Kix;
		ahrs_data.PIData.Kiz = ahrs_data.PIData.Kix;
#ifdef DEBUG_USB
		float32ToStr(ahrs_data.PIData.Kix, "Ki=", StringBuffer);
		sendUSBMessage(StringBuffer);
#endif
	}
	else
	{
		ahrs_data.PIData.Kix = DEFAULT_KI;
		ahrs_data.PIData.Kiy = DEFAULT_KI;
		ahrs_data.PIData.Kiz = DEFAULT_KI;

	#ifdef DEBUG_USB
		sendUSBMessage("Error reading Ki");
		sendUSBMessage("Using default values");
	#endif
	}

	if(strToFloat32(&(ahrs_data.PIData.Kpx), &FSBuffer[0], "Kpx=") == SUCCESS)
	{
		ahrs_data.PIData.Kpy = ahrs_data.PIData.Kpx;
		ahrs_data.PIData.Kpz = ahrs_data.PIData.Kpx;
#ifdef DEBUG_USB
		float32ToStr(ahrs_data.PIData.Kpx, "Kp=", StringBuffer);
		sendUSBMessage(StringBuffer);
#endif
	}
	else
	{
		ahrs_data.PIData.Kpx = DEFAULT_KP;
		ahrs_data.PIData.Kpy = DEFAULT_KP;
		ahrs_data.PIData.Kpz = DEFAULT_KP;

	#ifdef DEBUG_USB
		sendUSBMessage("Error reading Kp");
		sendUSBMessage("Using default values");
	#endif
	}
	//Close and unmount.
	f_close(&settingsFile);
	f_mount(0,0);

#ifdef DEBUG_USB
	sendUSBMessage("Settings loaded successfully");
#endif
	return SUCCESS;
}

void NVIC_EnableInterrupts(FunctionalState newState)
{
	//interrupt controller
	NVIC_InitTypeDef NVCInitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//init ADC interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = ADC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART1 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART1_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART2_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream6 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM8 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM14 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init I2C2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	if(newState)
	{
		SYSTEM_INTERRUPTS_ON = 1;
	}
	else
	{
		SYSTEM_INTERRUPTS_ON = 0;
	}
}

void extPeripheralInit(void)
{
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
	Delayms(1000);
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
	Delaynus(50000);
	// Reset PS
	PSReset();
	Delayms(1000);
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
	EXTSENS_NULLING_GYRO = 1;
#ifdef DEBUG_USB
	sendUSBMessage("Sensors configured");
#endif
	Delayms(1000);
#ifdef DEBUG_USB
	sendUSBMessage("Reset matrix");
#endif
	ahrs_resetRotationMatrix();
#ifdef DEBUG_USB
	sendUSBMessage("AutoPilot OnLine");
	sendUSBMessage("initialization done");
#endif
}

void Delayms(uint32_t ms)
{
	uint32_t time = systemTime - 1;
	uint32_t deltaTime = 0;
	LED_OK_OFF;
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
	LED_OK_ON;
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

float intToFloat(int whole, int frac)
{
	float result = 0;
	float temp = 0;
	temp = (float)frac;
	while(frac > 0)
	{
		frac = frac / 10;
	}
	result = (float)whole + temp;
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
