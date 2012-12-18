/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

//part of main branch

#include "allinclude.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/*
 * ===========================================================================
 *
 *  Abstract: main program
 *
 * ===========================================================================
 */

// USB related
__ALIGN_BEGIN	USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


int main(void)
{
	FATFS FileSystemObject;
	DSTATUS driveStatus;
	FIL logFile;
	unsigned int bytesWritten;
	//SD_CardInfo cardinfo;
	int i = 0;
	int registerCount = 0;
	// Startup delay - 0,5 sec
	Delaynus(500000);
	// Configure hardware
	System_Config();
	// Set LED OK = 1
	LED_OK_ON;
	// Set default PWM out values
	PWMOUT_1 = TIM1_PULSE;
	PWMOUT_2 = TIM1_PULSE;
	PWMOUT_3 = TIM1_PULSE;
	PWMOUT_4 = TIM1_PULSE;
	PWMOUT_5 = TIM1_PULSE;
	PWMOUT_6 = TIM1_PULSE;
	PWMOUT_7 = TIM1_PULSE;
	PWMOUT_8 = TIM1_PULSE;
	PWMOUT_9 = TIM1_PULSE;
	PWMOUT_10 = TIM1_PULSE;
	PWMOUT_11 = TIM1_PULSE;
	PWMOUT_12 = TIM1_PULSE;

	// Initialize USB
	  USBD_Init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS
	  USB_OTG_HS_CORE_ID,
	#else
	  USB_OTG_FS_CORE_ID,
	#endif
	  &USR_desc,
	  &USBD_HID_cb,
	  &USR_cb);

    while (1)
    {
        //Delaynus(1000000 / 2000);    /* A short delay */
        //Delaynus(1000000 / 2000);
        // Check MODBUS for messages
        if(MB_HASDATA)
        {
        	// Execute process data function
        	MODBUS_ExecuteFunction();
        	// Send data
        	// Enable DMA transfer
        	transferDMA_USART2(MODBUSData.bytes.cdata, MODBUSData.bytes.uiDataCount);
        	// Set MODBUS to IDLE
        	MB_SETTOIDLE;
        }

        //check SCR
        if(SCR1 & SCR_GETPSDATA)
        {
        	PSRequestData();
        	SCR1 = SCR1 & ~SCR_GETPSDATA;
        }
        if(SCR1 & SCR_SETPSI0)
        {
        	PSSetI0();
        	SCR1 = SCR1 & ~SCR_SETPSI0;
        }
        if(SCR1 & SCR_PSRESET)
        {
        	PSReset();
           	SCR1 = SCR1 & ~SCR_PSRESET;
        }
        if(SCR1 & SCR_GETGPSDATA)
        {
        	GPSSetDataOutput();
        	SCR1 = SCR1 & ~SCR_GETGPSDATA;
        }
        if(SCR1 & SCR_STOP_GPS)
        {
        	GPSStopOutput();
        	SCR1 = SCR1 & ~SCR_STOP_GPS;
        }
        if(SCR1 & SCR_INCPWM)
        {
        	TIM_SetCompare4(TIM1, PWMOUT_1);
        	SCR1 = SCR1 & ~SCR_INCPWM;
        }
        if(SCR1 & SCR_READI2C2)
        {
        	// Begin read of 21 registers from MPU6000
        	masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 22);
        	SCR1 = SCR1 & ~SCR_READI2C2;
        }
        if(SCR1 & SCR_WRITEI2C2)
        {

        	SCR1 = SCR1 & ~SCR_WRITEI2C2;
        }
        if(SCR1 & SCR_TESTI2C2AUTO)
        {
        	// Enable MPU
        	MPU6000_Enable(ENABLE);
        	// Enable I2C bypass to write to HMC5883
        	MPU6000_EnableI2CBypass(ENABLE);
        	// Configure HMC5883
        	HMC5883_Enable(ENABLE);
        	// Configure MPL3115A2
        	MPL3115A2_Enable(ENABLE);
        	// Test read HMC
        	//masterReceive_HMC5883L(HMC5883_ADDRESS, 3, I2C2_DMABufRX, 6);
        	// Test read MPL
        	//masterReceive(MPL3115A2_ADDRESS, 0, I2C2_DMABufRX, 45);
        	// Test read MPU
        	//masterReceive(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 21);
        	// Disable I2C bypass
        	MPU6000_EnableI2CBypass(DISABLE);
        	// Configure MPU I2C master mode
        	MPU6000_ConfigureI2CMaster();
        	// Enable MPU I2C master
        	MPU6000_EnableI2CMaster(ENABLE);

        	I2C2_INITDONE = 1;

        	SCR1 = SCR1 & ~SCR_TESTI2C2AUTO;
        }
        if(SCR1 & SCR_SET_PWM_0)
        {
        	PWMOUT_1 = 2100;
       		TIM_SetCompare4(TIM1, PWMOUT_1);
        	SCR1 = SCR1 & ~SCR_SET_PWM_0;
        }
        if(SCR1 & SCR_SET_PWM_PASSON)
        {
        	PWM_PASSTHROUGH = 1;
        	SCR1 = SCR1 & ~SCR_SET_PWM_PASSON;
        }
        if(SCR1 & SCR_SET_PWM_PASSOFF)
        {
        	PWM_PASSTHROUGH = 0;
        	SCR1 = SCR1 & ~SCR_SET_PWM_PASSOFF;
        }
        if(SCR1 & SCR_START_AD)
        {
        	ADC_ENABLED = ~ADC_ENABLED;
        	SCR1 = SCR1 & ~SCR_START_AD;
        }
        if(SCR1 & SCR_INIT_SENSORS)
        {
        	extPeripheralInit();
        	SCR1 = SCR1 & ~SCR_INIT_SENSORS;
        }
        if(SCR1 & SCR_DEC_DAC_FREQ)
        {
        	DAC1_TIM6reloadValue += 0xF;
        	if(DAC1_TIM6reloadValue > 0x11F)
        	{
        		DAC1_TIM6reloadValue = 0xFF;
        	}
        	TIM_SetAutoreload(TIM6, DAC1_TIM6reloadValue);
        	SCR1 = SCR1 & ~SCR_DEC_DAC_FREQ;
        }
        if(SCR1 & SCR_TESD_SD)
        {
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
				FSBuffer[21] = 't';
				FSBuffer[22] = 'x';
				FSBuffer[23] = 't';
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
        	f_write(&logFile, "val1=1234;\n", 10, &bytesWritten);
        	//Flush the write buffer with f_sync(&logFile);
        	//f_sync(&logFile);
        	//Close and unmount.
        	f_close(&logFile);
        	f_mount(0,0);

        	// Test read file
        	loadSettings();

        	SCR1 = SCR1 & ~SCR_TESD_SD;
        }

        // Send registers 0 - 30
        if(USB_REQUEST_DATA_0)
        {
        	USB_REQUEST_DATA_0 = 0;
        	// Set register pointer
        	registerCount = 0;
        	// Store data
        	Buffer[0] = 2;
        	Buffer[1] = 0;
        	for(i=2;i<64;i+=2)
        	{
        		// Store high byte of register
        		Buffer[i] = (MODBUSReg[registerCount] >> 8) & 0x00FF;
        		// Store low byte of register
        		Buffer[i + 1] = MODBUSReg[registerCount] & 0x00FF;
        		registerCount++;
        	}
        	USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64);
        }

        // Send registers 31 - 61
        if(USB_REQUEST_DATA_1)
        {
        	USB_REQUEST_DATA_1 = 0;
        	// Set register pointer
        	registerCount = 31;
        	// Store data
        	Buffer[0] = 2;
        	Buffer[1] = 1;
        	for(i=2;i<64;i+=2)
        	{
        		// Store high byte of register
        		Buffer[i] = (MODBUSReg[registerCount] >> 8) & 0x00FF;
        		// Store low byte of register
        		Buffer[i + 1] = MODBUSReg[registerCount] & 0x00FF;
        		registerCount++;
        	}
        	USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64);
        }

        // Send registers 62 - 79
        if(USB_REQUEST_DATA_2)
        {
        	USB_REQUEST_DATA_2 = 0;
        	// Set register pointer
        	registerCount = 62;
        	// Store data
        	Buffer[0] = 2;
        	Buffer[1] = 2;
        	for(i=2;i<36;i+=2)
        	{
        		// Store high byte of register
        		Buffer[i] = (MODBUSReg[registerCount] >> 8) & 0x00FF;
        		// Store low byte of register
        		Buffer[i + 1] = MODBUSReg[registerCount] & 0x00FF;
        		registerCount++;
        	}
        	USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64);
        }

        // Check PS for messages
        if(PS_HASDATA)
        {
        	// Execute process data function
        	processPSData();
        	// Set PS process to IDLE
        	PS_SETIDLE;
        	// Mark PS not busy
        	PSBUSY = 0;
        	// Mark sensor OK
        	SCR2 = SCR2 | SCR2_POWEROK;
        }
        // Check I2C data
        if(COPYI2C)
        {
        	copySensorData();
        	// Mark sensors OK
        	SCR2 = SCR2 | SCR2_ACCOK;
        	SCR2 = SCR2 | SCR2_GYROOK;
        	SCR2 = SCR2 | SCR2_MAGOK;
        	SCR2 = SCR2 | SCR2_BAROK;
        }
    }
    /* Infinite loop */
    while (1)
    {
    }
}

