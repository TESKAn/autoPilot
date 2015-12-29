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
	unsigned int bytesWritten;
	//float32_t temp = 0;

	// Init sensor data structures
	fusion_init(&fusionData);

	// Setup RC PWM outputs
	RCData.PWMOUT_1 = 1500;
	RCData.PWMOUT_2 = 1500;
	RCData.PWMOUT_3 = 1500;
	RCData.PWMOUT_4 = 1500;
	RCData.PWMOUT_5 = 1500;
	RCData.PWMOUT_6 = 1500;
	RCData.PWMOUT_7 = 1500;
	RCData.PWMOUT_8 = 1500;
	RCData.PWMOUT_9 = 1500;
	RCData.PWMOUT_10 = 1500;
	RCData.PWMOUT_11 = 1500;
	RCData.PWMOUT_12 = 1500;

	//PWM_PASSTHROUGH = 1;

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
	// Configure hardware
	System_Config();
	// Set default PWM out values
	refreshPWMOutputs();

	globalVar = 0.001f;

	// Set UART buffer
	UART_Init();

	// Init RS485
	RS485_MasterInitData();


	// Peripherals initialized, wait 1 sec
	Delayms(100);


	// Initialize flight data
	flight_init(&FCFlightData, &RCData);

	// Initialize external peripheral
	extPeripheralInit();

	// Calibrate I2C sensors
	// Function has no function...
	calibrateI2CSensors();

	PWMEN_OUT_ENABLE = 1;


	MPU_COMM_ENABLED = 1;

	RS485_RXEN;

	// Mount SD card
    while (1)
    {
    	// Main loop switch
    	switch(mainLoopState)
    	{
			case 0:
			{
				break;
			}
			case 1:
			{
				mainLoopState = 0;
				// Reset DCM
				// Create identity matrix
				matrix3_init(1, &fusionData._fusion_DCM);
				matrix3_init(1, &fusionData._GPS_DCM);
				// Reset PIDs
				math_PID3Reset(&fusionData._gyroErrorPID);
				// Set initial value
				fusion_initGyroDriftPID(&fusionData);
				fusion_initGyroGainPID(&fusionData);

				break;
			}
			case 2:
			{
				mainLoopState = 0;
				// Stop comm
				MPU_COMM_ENABLED = 0;
				// Wait if there is transmission in progress
				while(I2C2_WAITINGDATA ) {}
				MPU6000_GyroSelfTest(ENABLE);
				// Enable comm
				MPU_COMM_ENABLED = 1;

				break;
			}
			case 3:
			{
				mainLoopState = 0;
				MPU_COMM_ENABLED = 0;
				while(I2C2_WAITINGDATA ) {}
				MPU6000_GyroSelfTest(DISABLE);
				MPU_COMM_ENABLED = 1;

				break;
			}
			case 4:
			{
				mainLoopState = 0;
				MPU_COMM_ENABLED = 0;
				while(I2C2_WAITINGDATA ) {}
				MPU6000_AccSelfTest(ENABLE);
				MPU_COMM_ENABLED = 1;

				break;
			}
			case 5:
			{
				mainLoopState = 0;
				MPU_COMM_ENABLED = 0;
				while(I2C2_WAITINGDATA ) {}
				MPU6000_AccSelfTest(DISABLE);
				MPU_COMM_ENABLED = 1;

				break;
			}
			case 6:
			{
				mainLoopState = 0;
				MPU_COMM_ENABLED = 0;
				while(I2C2_WAITINGDATA ) {}
				MPU6000_ReadFTValues();
				MPU_COMM_ENABLED = 1;


				break;
			}
			case 7:
			{
				mainLoopState = 0;
				// Acc offsets
				acc_updateOffsets(&fusionData._accelerometer);
				break;
			}
			case 8:
			{
				// All gains
				acc_updateGains(&fusionData._accelerometer, 0);
				mainLoopState = 0;
				break;
			}
			case 9:
			{
				// X gain
				acc_updateGains(&fusionData._accelerometer, 1);
				mainLoopState = 0;
				break;
			}
			case 10:
			{
				// Y gain
				acc_updateGains(&fusionData._accelerometer, 2);
				mainLoopState = 0;
				break;
			}
			case 11:
			{
				// Z gain
				acc_updateGains(&fusionData._accelerometer, 3);
				mainLoopState = 0;
				break;
			}
			case 12:
			{
				SENSOR_TEST_STEP = 1;
				mainLoopState = 0;
				break;
			}
			case 13:
			{
				UART_QueueMessagef(VAR_GYRO_X, fusionData._gyro.vector.x);
				mainLoopState = 0;
				break;
			}
			case 14:
			{
				// Send data with DMA
				// Torque ON
				RS485_ServoTest(0);
				mainLoopState = 0;
				break;
			}
			case 15:
			{
				// Send data with DMA
				// Torque OFF
				RS485_ServoTest(1);
				mainLoopState = 0;
				break;
			}
			case 16:
			{
				// Send data with DMA
				// Set ID to 0x11
				RS485_ServoTest(2);
				mainLoopState = 0;
				break;
			}
			case 17:
			{
				// Send data with DMA
				// Set FR position
				RS485_ServoTest(3);
				mainLoopState = 0;
				break;
			}
			case 18:
			{
				// Start servo poll
				RS485_MasterState(1);
				mainLoopState = 0;
				break;
			}
			case 19:
			{
				// Stop servo poll
				RS485_MasterState(2);
				mainLoopState = 0;
				break;
			}
			case 20:
			{
				// Buffer servo move
				RS485_SetupServos();
				mainLoopState = 0;
				break;
			}
			case 21:
			{
				RS485_ServoTest(4);
				mainLoopState = 0;
				break;
			}
			case 22:
			{
				RS485_ServoTest(5);
				mainLoopState = 0;
				break;
			}
			case 23:
			{
				RS485_ServoTest(6);
				mainLoopState = 0;
				break;
			}
			default:
			{
				mainLoopState = 0;
				break;
			}
    	}

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

