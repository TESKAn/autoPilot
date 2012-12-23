/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"
#include <string.h>

void openLog(void)
{
	unsigned int bytesWritten;
	// Open file for writing
	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
		//flag error
	}
	driveStatus = disk_status (0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT)
		   )
	{
		//flag error.
	}
	// Generate file name
	// File name = "/LOG_ddmmyyyy_hhmmss.txt"
	FSBuffer = malloc(25);
	// Check if malloc returned OK
	if(FSBuffer != (char*)-1)
	{
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
		}

		// Free memory
		free(FSBuffer);
	}
	else
	{
		// Open file
		if(f_open(&logFile, "/LOG_ddmmyyyy_hhmmss.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
		{
			//flag error
		}
	}
	// Write first line
	f_write(&logFile, ";AccX;AccY;AccZ;GyroX;GyroY;GyroZ;MagX;MagY;MagZ;Baro;Voltage;Current;mAh;T1;T2;T3\r\n", 83, &bytesWritten);
}

void closeLog(void)
{
	//Close and unmount.
	SCR2 = SCR2 & ~SCR2_LOGOPEN;
	SD_WRITE_LOG = 0;
	f_close(&logFile);
	f_mount(0,0);
}

void write_toLog(void)
{
	unsigned int bytesWritten;
	unsigned int uiByteCount = 0;
	DEBUG_PIN_ON;
	FSBuffer = malloc(128);
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

	// Free memory
	free(FSBuffer);

	DEBUG_PIN_OFF;
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

uint16_t strToNumber(char* file, char* str)
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
	strBeginning = strpbrk(file, str);
	// Check that it is not null
	if(strBeginning != NULL)
	{
		// Get pointer to where in file string ends
		strEnd = strpbrk(strBeginning, ";");
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

void loadSettings(void)
{
	FATFS FileSystemObject;
	DSTATUS driveStatus;
	FIL settingsFile;
	unsigned int bytesToRead = 0;
	unsigned int readBytes = 0;

	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
		//flag error
	}
	driveStatus = disk_status (0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT)
		   )
	{
		//flag error.
	}
	// Open file
	if(f_open(&settingsFile, "/settings.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		//flag error
	}
	// Read file to buffer
	bytesToRead = f_size(&settingsFile);
	FSBuffer = malloc(bytesToRead);

	if(f_read (&settingsFile, FSBuffer, bytesToRead, &readBytes) != FR_OK)
	{

	}
	// Process data
	SD_RESULT = strToNumber(FSBuffer, "val1=");

	//Close and unmount.
	f_close(&settingsFile);
	f_mount(0,0);
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
	// Wait some time
	Delaynus(50000);
	// Set LED OK = 1
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Do a long delay, ca. 1 sec
	Delaynus(1000000);
	// set PS busy
	PSBUSY = 1;
	// Set I0 current for sensor
	PSSetI0();
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Reset PS
	PSReset();
	LED_OK_OFF;
	// Long delay
	Delaynus(2000000);
	// Set LED OK = 0
	LED_OK_ON;
	// Reset PS busy
	PSBUSY = 0;
	// Configure GPS
	GPSSetDataOutput();
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Disable interrupts before configuring I2C
	NVIC_EnableInterrupts(DISABLE);
	// Configure I2C sensors
	sensorInit();
	// Enable ADC
	ADC_ENABLED = 1;
	// Mark sensors initiated
	EXTSENS_INIT_DONE = 1;
	// Reenable interrupts
	NVIC_EnableInterrupts(ENABLE);
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
