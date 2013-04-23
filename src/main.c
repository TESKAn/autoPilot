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

#include "AudioComm.h"

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
USB_OTG_CORE_HANDLE  USB_OTG_dev;

// AHRS
float32_t t1 = 0;
float32_t t2 = 0;

int main(void)
{
	unsigned int bytesWritten;
	float32_t temp = 0;
	// Init vectors
	// Initialize matrices
	ahrs_matrix3by3_init(&tempMatrix);

	ahrs_matrix3by3_init(&holdMatrix);
	math_vector3fDataInit(&tempVector, ROW);
	math_vector3fDataInit(&tempVector1, ROW);
	math_vector3fDataInit(&tempVector2, ROW);

	AHRS_FIRSTRUN_PID = 1;
	AHRS_FIRSTRUN_MATRIX = 1;
	TIMEOUT_USEINTERRUPT = 1;

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

	globalVar = 0.001f;

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
	// Peripherals initialized, wait 1 sec
	Delayms(100);

	// Initialize external peripheral
	extPeripheralInit();

	AC_InitBuffers();
	AC_RDisparity = -1;
	// Put a message in buffer to test audio communication
	AC_MESSAGE_IN_BUFFER = 1;
	ACMessage[0] = 0xaa;
	ACMessage[1] = 0x55;
	ACMessageLength = 2;

	PWMEN_OUT_ENABLE = 1;

    while (1)
    {
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
        if(SCR1 & SCR_01)
        {
        	GPSSetDataOutput();
        	SCR1 = SCR1 & ~SCR_01;
        }
        if(SCR1 & SCR_02)
        {
         	GPSStopOutput();
        	SCR1 = SCR1 & ~SCR_02;
        }
        if(SCR1 & SCR_03)
        {
        	PWM_PASSTHROUGH = 1;
           	SCR1 = SCR1 & ~SCR_03;
        }
        if(SCR1 & SCR_04)
        {
        	PWM_PASSTHROUGH = 0;
        	SCR1 = SCR1 & ~SCR_04;
        }
        if(SCR1 & SCR_05)
        {
        	extPeripheralInit();
        	SCR1 = SCR1 & ~SCR_05;
        }
        if(SCR1 & SCR_06)
        {
        	if(!SD_WRITE_LOG)
        	{
				SD_WRITE_LOG = 1;
				SCR2 = SCR2 | SCR2_LOGOPEN;
				// Check if log file is open
				if(!SD_LOG_ISOPEN)
				{
					openLog();
				}
        	}
        	else
        	{
        		SD_WRITE_LOG = 0;
        		SCR2 = SCR2 & ~SCR2_LOGOPEN;
        		// Else close file
        		closeLog();
        	}
        	SCR1 = SCR1 & ~SCR_06;
        }
        if(SCR1 & SCR_07)
        {
        	EXTSENS_NULLING_ACC = 1;
        	SCR1 = SCR1 & ~SCR_07;
        }
        if(SCR1 & SCR_08)
        {
        	EXTSENS_FS_REPORT = 1;
        	SCR1 = SCR1 & ~SCR_08;
        }
        if(SCR1 & SCR_09)
        {
        	EXTSENS_FS_REPORT = 0;
        	SCR1 = SCR1 & ~SCR_09;
        }
        if(SCR1 & SCR_10)
        {
        	ahrs_resetPID(&(ahrs_data.PIData));
        	AHRS_FIRSTRUN_PID = 1;
        	SCR1 = SCR1 & ~SCR_10;
        }
        if(SCR1 & SCR_11)
        {
        	// Reset AHRS matrix
        	ahrs_resetRotationMatrix();
        	ahrs_resetQuaternion();
        	AHRS_FIRSTRUN_MATRIX = 1;
        	SCR1 = SCR1 & ~SCR_11;
        }
        if(SCR1 & SCR_12)
        {
        	//loadSettings();
        	//loadSingleSetting("hardMagZ=", &temp);
        	//PWMEN_OUT_ENABLE = ~PWMEN_OUT_ENABLE;
        	SD_POWER_TOGGLE;
        	SCR1 = SCR1 & ~SCR_12;
        }
        if(SCR1 & SCR_13)
        {
        	temp = ahrs_data.PIData.Kix / 10 + 0.0005f;
        	ahrs_data.PIData.Kix = ahrs_data.PIData.Kix + temp;
        	ahrs_data.PIData.Kiy = ahrs_data.PIData.Kiy + temp;
        	ahrs_data.PIData.Kiz = ahrs_data.PIData.Kiz + temp;
    		float32ToStr(ahrs_data.PIData.Kix, "Ki=", StringBuffer);
    		sendUSBMessage(StringBuffer);
        	SCR1 = SCR1 & ~SCR_13;
        }
        if(SCR1 & SCR_14)
        {
        	temp = ahrs_data.PIData.Kix / 10 + 0.0005f;
        	ahrs_data.PIData.Kix = ahrs_data.PIData.Kix - temp;
        	ahrs_data.PIData.Kiy = ahrs_data.PIData.Kiy - temp;
        	ahrs_data.PIData.Kiz = ahrs_data.PIData.Kiz - temp;
    		float32ToStr(ahrs_data.PIData.Kix, "Ki=", StringBuffer);
    		sendUSBMessage(StringBuffer);
        	SCR1 = SCR1 & ~SCR_14;
        }
        if(SCR1 & SCR_15)
        {
        	temp = ahrs_data.PIData.Kpx / 10 + 0.0005f;
        	ahrs_data.PIData.Kpx = ahrs_data.PIData.Kpx + temp;
        	ahrs_data.PIData.Kpy = ahrs_data.PIData.Kpy + temp;
        	ahrs_data.PIData.Kpz = ahrs_data.PIData.Kpz + temp;
        	//globalVar = globalVar + 0.0005f;
    		//float32ToStr(globalVar, "GV=", StringBuffer);
        	float32ToStr(ahrs_data.PIData.Kpx, "Kp=", StringBuffer);
    		sendUSBMessage(StringBuffer);
        	SCR1 = SCR1 & ~SCR_15;
        }
        if(SCR1 & SCR_16)
        {
        	temp = ahrs_data.PIData.Kpx / 10 + 0.0005f;
        	ahrs_data.PIData.Kpx = ahrs_data.PIData.Kpx - temp;
        	ahrs_data.PIData.Kpy = ahrs_data.PIData.Kpy - temp;
        	ahrs_data.PIData.Kpz = ahrs_data.PIData.Kpz - temp;
        	//globalVar = globalVar - 0.0005f;
    		//float32ToStr(globalVar, "GV=", StringBuffer);
        	float32ToStr(ahrs_data.PIData.Kpx, "Kp=", StringBuffer);
    		sendUSBMessage(StringBuffer);
        	SCR1 = SCR1 & ~SCR_16;
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

        // Send registers 62 - 90
        if(USB_REQUEST_DATA_2)
        {
        	USB_REQUEST_DATA_2 = 0;
        	// Set register pointer
        	registerCount = 62;
        	// Store data
        	Buffer[0] = 2;
        	Buffer[1] = 2;
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
        	// Mark sensors OK
        	SCR2 = SCR2 | SCR2_ACCOK;
        	SCR2 = SCR2 | SCR2_GYROOK;
        	SCR2 = SCR2 | SCR2_MAGOK;
        	SCR2 = SCR2 | SCR2_BAROK;
        }
        // Check log write buffers
        if(SD_WRITING_BUF1)
        {
        	//getFTime(&t1);
        	f_write(&logFile, SD_Buffer1, SD_Buf1Count, &bytesWritten);
        	//getFTime(&t2);
        	SD_Buf1Count = 0;
        	SD_WRITING_BUF1 = 0;
        }
        if(SD_WRITING_BUF2)
        {
        	f_write(&logFile, SD_Buffer2, SD_Buf2Count, &bytesWritten);
        	SD_Buf2Count = 0;
        	SD_WRITING_BUF2 = 0;
        }
    }
    /* Infinite loop */
    while (1)
    {
    }
}

