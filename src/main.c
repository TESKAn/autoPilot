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

#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"

#include "sensors/sensors_fusion.h"



/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

uint16_t MavlinkTransfer();

/*
 * ===========================================================================
 *
 *  Abstract: main program
 *
 * ===========================================================================
 */


// AHRS
float32_t t1 = 0;
float32_t t2 = 0;



int main(void)
{
	//uint16_t ui16Temp = 0;
	unsigned int bytesWritten;
	//uint32_t ui32Temp = 0;
	SYSTEM_RUNNING = 0;
	//float32_t temp = 0;
#ifdef USE_FREEMASTER
	FMSTR_Init();
#endif

	// Init sensor data structures
	fusion_init(&fusionData, getSystemTime());

	// Init MAVLINK buffer
	RB32_Init(&rb32MavlinkTXQueue, ui32MavlinkBuf, 32);

	// Initialize flight data
	flight_init(&FCFlightData, &RCData);

	// Set motor IDs
	FCFlightData.MOTORS.FR.ui8MotorID = CAN_MOTOR_FR_ID;
	FCFlightData.MOTORS.FL.ui8MotorID = CAN_MOTOR_FL_ID;
	FCFlightData.MOTORS.RR.ui8MotorID = CAN_MOTOR_RR_ID;
	FCFlightData.MOTORS.RL.ui8MotorID = CAN_MOTOR_RL_ID;

	AHRS_FIRSTRUN_PID = 1;
	AHRS_FIRSTRUN_MATRIX = 1;
	TIMEOUT_USEINTERRUPT = 1;

	//SD_CardInfo cardinfo;
	/*
	int i = 0;
	int j = 0;
	int registerCount = 0;*/
	// Startup delay - 0,5 sec
	Delaynus(500000);

	// Configure MCU hardware
	System_Config();

	// Init CAN filters
	InitCANLink();

	// Set default PWM out values
	refreshPWMOutputs();

	// Peripherals initialized, wait 1 sec
	Delayms(100);

	// Initialize external peripheral
	extPeripheralInit();

	// Calibrate I2C sensors
	// Function has no function...
	//calibrateI2CSensors();

	PWMEN_OUT_ENABLE = 1;

	MPU_COMM_ENABLED = 1;

	SYSTEM_RUNNING = 1;

    while (1)
    {
    	// Check freemaster
    	FMSTR_Poll();

    	// Check MAVLINK
    	if(0 == mavlinkSendBusy)
    	{
    		MavlinkTransfer();
    	}

    	// Check main loop
    	CheckMainLoopStates();

        // Check log write buffers
        if(SD_WRITING_BUF1)
        {
        	//getFTime(&t1);
        	LED_RUN_ON;
        	f_write(&logFile, SD_Buffer1, SD_Buf1Count, &bytesWritten);
        	LED_RUN_OFF;
        	//getFTime(&t2);
        	SD_Buf1Count = 0;
        	SD_WRITING_BUF1 = 0;
        }
        if(SD_WRITING_BUF2)
        {
        	LED_RUN_ON;
        	f_write(&logFile, SD_Buffer2, SD_Buf2Count, &bytesWritten);
        	LED_RUN_OFF;
        	SD_Buf2Count = 0;
        	SD_WRITING_BUF2 = 0;
        }
    }
    /* Infinite loop */
    while (1)
    {
    }
}

