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
	uint16_t ui16Temp = 0;
	unsigned int bytesWritten;
	uint32_t ui32Temp = 0;
	SYSTEM_RUNNING = 0;
	//float32_t temp = 0;
#ifdef USE_FREEMASTER
	FMSTR_Init();
#endif

	// Init sensor data structures
	fusion_init(&fusionData, getSystemTime());

	// Init serial port data structure
	RB_Init(&RB_USART3, usart3_buf, 128);
	RB_Init(&RB_USART2, usart2_buf, 128);

	// Init MAVLINK buffer
	RB32_Init(&rb32MavlinkTXQueue, ui32MavlinkBuf, 32);


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

	// Init CAN filters
	InitCANLink();

	// Set default PWM out values
	refreshPWMOutputs();

	globalVar = 0.001f;

	// Peripherals initialized, wait 1 sec
	Delayms(100);

	// Initialize flight data
	flight_init(&FCFlightData, &RCData);

	// Set motor IDs
	FCFlightData.MOTORS.FR.ui8MotorID = CAN_MOTOR_FR_ID;
	FCFlightData.MOTORS.FL.ui8MotorID = CAN_MOTOR_FL_ID;
	FCFlightData.MOTORS.RR.ui8MotorID = CAN_MOTOR_RR_ID;
	FCFlightData.MOTORS.RL.ui8MotorID = CAN_MOTOR_RL_ID;


	// Initialize external peripheral
	extPeripheralInit();

	// Calibrate I2C sensors
	// Function has no function...
	calibrateI2CSensors();

	PWMEN_OUT_ENABLE = 1;


	MPU_COMM_ENABLED = 1;

	RS485_RXEN;

	// Start servo poll
	//RS485_MasterState(1);



	SYSTEM_RUNNING = 1;
	// Mount SD card
    while (1)
    {
    	/*
    	// Check CAN RX interrupts
    	if(!(CAN1->IER & CAN_IT_FMP0))
    	{
    		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    	}
    	if(!(CAN1->IER & CAN_IT_FMP1))
    	{
    		CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
    	}*/



#ifndef USE_FREEMASTER
    	// Process serial comm
    	if(0 != RB_USART2.count)
    	{
    		UART_RcvData(RB_pop(&RB_USART2));
    	}
    	if(COMM_SEND_DATA)
    	{
    		if(0 == UART2_Transferring)
    		{
				COMM_SEND_DATA = 0;
				LED_ToggleCount = 0;
				switch(ui8BufferToSend)
				{
					case 0:
					{
						SendCommData();
						ui8BufferToSend = 1;
						break;
					}
					case 1:
					{
						SendCommData1();
						ui8BufferToSend = 0;
						break;
					}
					default:
					{
						ui8BufferToSend = 0;
						break;
					}
				}
    		}
    	}
#else
    	// Check freemaster
    	FMSTR_Poll();
#endif

    	// Check MAVLINK
    	if(0 == mavlinkSendBusy)
    	{
    		if(rb32MavlinkTXQueue.count)
    		{
    			ui32Temp = RB32_pop(&rb32MavlinkTXQueue);
    			switch(ui32Temp)
    			{
					case 0:
					{
						ui16Temp = mavlink_msg_heartbeat_pack(1, 1, &mavlinkMessageDataTX, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
						ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
						transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
						mavlinkSendBusy = 1;
						break;
					}
					case 1:
					{
						ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 2500, FCFlightData.batMon.ui16MavlinkBatteryVoltages, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1, -1, MAV_BATTERY_CHARGE_STATE_OK);
						ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
						transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
						mavlinkSendBusy = 1;
						break;
					}
					case 2:
					{
						ui16Temp = mavlink_msg_attitude_pack(1, 1,&mavlinkMessageDataTX, (uint32_t)getFTime(), fusionData.ROLLPITCHYAW.roll, fusionData.ROLLPITCHYAW.pitch, fusionData.ROLLPITCHYAW.yaw, fusionData.updateRotation.x, fusionData.updateRotation.y, fusionData.updateRotation.z);
						ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
						transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
						mavlinkSendBusy = 1;
						break;
					}
					default:
					{
						break;
					}
    			}
    		}
    	}


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
				// Start servo poll
				//RS485_MasterState(1);
				mainLoopState = 0;
				break;
			}
			case 13:
			{
				// Stop servo poll
				//RS485_MasterState(2);
				mainLoopState = 0;
				break;
			}
			case 14:
			{
				// Enable motor
				//RS485_MotorTest(0);
				mainLoopState = 0;
				break;
			}
			case 15:
			{
				// Disable motor
				//RS485_MotorTest(1);
				mainLoopState = 0;
				break;
			}
			case 16:
			{
				// Set RPM
				//RS485_MotorTest(2);
				mainLoopState = 0;
				break;
			}
			case 17:
			{
				// Store command
				//RS485_QueueCommand(RS485ExecuteCommand);
				mainLoopState = 0;
				break;
			}
			case 18:
			{
				refreshPWMOutputs();
				mainLoopState = 0;
				break;
			}
			case 19:
			{
				CAN_SendMinMaxRPM();
				mainLoopState = 0;
				break;
			}
			case 20:
			{
				CAN_SendRPM(2000, 2000, 0);
				mainLoopState = 0;
				break;
			}
			case 21:
			{
				CAN_SendRPM(2000, 2000, 1);
				mainLoopState = 0;
				break;
			}
			case 22:
			{
				mainLoopState = 0;
				break;
			}
			case 23:
			{
				init_CAN();
				mainLoopState = 0;
				break;
			}
			case 24:
			{
				CAN_SendENABLE(1, CAN_MOTOR_ALL_ID);
				mainLoopState = 0;
				break;
			}
			case 25:
			{
				CAN_SendENABLE(0, CAN_MOTOR_ALL_ID);
				mainLoopState = 0;
				break;
			}
			case 26:
			{
				if(0 == mavlinkSendBusy)
				{
					/*
					//ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 1, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 25, &FCFlightData.batMon.ui6MavLinkVoltage, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1, -1, 	MAV_BATTERY_CHARGE_STATE_OK);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);*/
					mavlinkSendBusy = 1;
				}
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

