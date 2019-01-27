/*
 * events.c
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#include "allinclude.h"
/*
  * @brief  This function handles ADC event interrupt request.
  * @param  None
  * @retval None
  * @services ADC
  */
void ADC_ISR_Handler(void)
{
	uint16_t result = 0;
	float32_t fTemp;
	// Check interrupt source - ADC1, ADC2 or ADC3
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != (u16)RESET)
	{
		// Channel 1
		// Get conversion result
		result = ADC_GetConversionValue(ADC1);
		// Store value
		AIn0 = result;
		// Clear flag
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		// Clear interrupt
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
	if(ADC_GetITStatus(ADC2, ADC_IT_EOC) != (u16)RESET)
	{
		// Channel 2
		fTemp = getFTime();
		FCFlightData.batMon.fUTime = fTemp - FCFlightData.batMon.fUTime_m;
		FCFlightData.batMon.fUTime_m = fTemp;
		// Get conversion result
		result = ADC_GetConversionValue(ADC2);
		// Store value
		AIn2 = result;
		FCFlightData.batMon.fUBat = (float32_t)result;
		FCFlightData.batMon.fUBat *= 0.015f;
		FCFlightData.batMon.fUBat += 0.068f;
		FCFlightData.batMon.ui6MavLinkVoltage = (uint16_t)(FCFlightData.batMon.fUBat * 1000.0f);
		// Clear flag
		ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
		// Clear interrupt
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	}
	if(ADC_GetITStatus(ADC3, ADC_IT_EOC) != (u16)RESET)
	{
		// Channel 3
		fTemp = getFTime();
		FCFlightData.batMon.fITime = fTemp - FCFlightData.batMon.fITime_m;
		FCFlightData.batMon.fITime_m = fTemp;
		// Get conversion result
		result = ADC_GetConversionValue(ADC3);
		// Store value
		AIn3 = result;
		FCFlightData.batMon.fIBat = (float32_t)result;
		FCFlightData.batMon.fIBat *= -0.0394f;
		FCFlightData.batMon.fIBat += 126.03f;
		FCFlightData.batMon.i16MavLinkCurrent = (int16_t)(FCFlightData.batMon.fIBat * 1000.0f);
		// Clear flag
		ADC_ClearFlag(ADC3, ADC_FLAG_EOC);
		// Clear interrupt
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);

		// To get mA, mul I * 1000; time is 1 ms, so div by 1000.
		FCFlightData.batMon.fmAsUsed += FCFlightData.batMon.fIBat;
		FCFlightData.batMon.fmAhUsed = FCFlightData.batMon.fmAsUsed / 3600.0f;
		FCFlightData.batMon.i32MavLinkCurrentConsumed = (int32_t)(FCFlightData.batMon.fmAsUsed);
	}
}

/**
  * @brief  This function handles DMA1 stream6 event interrupt request, UART2.
  * @param  None
  * @retval None
  * @services DMA1 stream 6
  */

void DMA1_Stream6_ISR_Handler(void)
{
	// Clear DMA interrupt
	DMA_ClearITPendingBit(DMA_USART2, DMA_IT_TC);
	DMA_ITConfig(DMA_USART2, DMA_IT_TC, DISABLE);
	// Enable UART2 TC interrupt to wait for end of transfer
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
}

/**
  * @brief  This function handles DMA1 stream4 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 4
  */

void DMA1_Stream4_ISR_Handler(void)
{
	// Clear interrupt flag
	USART_ClearFlag(USART3, USART_FLAG_TC);
	// Enable USART1 TC interrupt
	USART_ITConfig(USART3, USART_IT_TC, ENABLE);
	// Clear DMA interrupt
	DMA_ClearITPendingBit(DMA_USART3, DMA_IT_TC);
	DMA_ITConfig(DMA_USART3, DMA_IT_TC, DISABLE);

}

/**
  * @brief  This function handles DMA1 stream3 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 3
  */
void DMA1_Stream3_ISR_Handler(void)
{
	// Disable interrupts
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, DISABLE);
	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= DISABLE)
	{}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TC);
	// Copy data
	copySensorData();
	// Set flag to mark has data
	COPYI2C = 1;
	// Clear receive in progress
	I2C2_WAITINGDATA = 0;
}

/**
  * @brief  This function handles DMA2 stream5 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA5 stream 5
  */

void DMA2_Stream7_ISR_Handler(void)
{
	// Clear mavlink is sending data
	mavlinkSendBusy = 0;
	//GPS_SENDING = 0;
	DMA_ClearITPendingBit(DMA_USART1, DMA_IT_TC);
	DMA_ITConfig(DMA_USART1, DMA_IT_TC, DISABLE);
}


/**
  * @brief  This function handles Timer 4 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM4
  */
void TIM4_ISR_Handler(void)
{
	uint32_t dataTemp = 0;
	uint32_t result = 0;
	if((TIM4->SR & TIM_FLAG_CC1) != (u16)RESET)
	{
		// CC on channel 1
		dataTemp = TIM_GetCapture1(TIM4);
		// Calculate time
		// Check value
		if(dataTemp > TIM4_IC1_PreviousValue)
		{
			result = dataTemp - TIM4_IC1_PreviousValue;
		}
		else
		{
			result = (TIM4_PERIOD - TIM4_IC1_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) != 0)
		{
			// Input is not 0, transition from low to high
			TIM4_IC1_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM4_IC1_HighWidth = result;
			RCData.ch[0].PWMIN = (uint16_t)TIM4_IC1_HighWidth;
			RCData.ch[0].PWM_Good = 1;
			RCData.ch[0].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare4(TIM1, RCData.ch[0].PWMIN);
				RCData.ch[0].PWMOUT = RCData.ch[0].PWMIN;
			}
		}
		TIM4_IC1_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		TIM_ClearFlag(TIM4, TIM_FLAG_CC1);
	}

	else if((TIM4->SR & TIM_FLAG_CC2) != (u16)RESET)
	{
		// CC on channel 2
		dataTemp = TIM_GetCapture2(TIM4);
		// Calculate time
		// Check value
		if(dataTemp > TIM4_IC2_PreviousValue)
		{
			result = dataTemp - TIM4_IC2_PreviousValue;
		}
		else
		{
			result = (TIM4_PERIOD - TIM4_IC2_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13) != 0)
		{
			// Input is not 0, transition from low to high
			TIM4_IC2_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM4_IC2_HighWidth = result;
			RCData.ch[1].PWMIN = (uint16_t)TIM4_IC2_HighWidth;
			RCData.ch[1].PWM_Good = 1;
			RCData.ch[1].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare3(TIM1, RCData.ch[1].PWMIN);
				RCData.ch[1].PWMOUT = RCData.ch[1].PWMIN;
			}
		}
		TIM4_IC2_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		TIM_ClearFlag(TIM4, TIM_FLAG_CC2);
	}

	else if((TIM4->SR & TIM_FLAG_CC3) != (u16)RESET)
	{
		// CC on channel 3
		dataTemp = TIM_GetCapture3(TIM4);
		// Calculate time
		// Check value
		if(dataTemp > TIM4_IC3_PreviousValue)
		{
			result = dataTemp - TIM4_IC3_PreviousValue;
		}
		else
		{
			result = (TIM4_PERIOD - TIM4_IC3_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14) != 0)
		{
			// Input is not 0, transition from low to high
			TIM4_IC3_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM4_IC3_HighWidth = result;
			RCData.ch[2].PWMIN = (uint16_t)TIM4_IC3_HighWidth;
			RCData.ch[2].PWM_Good = 1;
			RCData.ch[2].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare2(TIM1, RCData.ch[2].PWMIN);
				RCData.ch[2].PWMOUT = RCData.ch[2].PWMIN;
			}
		}
		TIM4_IC3_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
		TIM_ClearFlag(TIM4, TIM_FLAG_CC3);
	}

	else if((TIM4->SR & TIM_FLAG_CC4) != (u16)RESET)
	{
		// CC on channel 4
		dataTemp = TIM_GetCapture4(TIM4);
		// Calculate time
		// Check value
		if(dataTemp > TIM4_IC4_PreviousValue)
		{
			result = dataTemp - TIM4_IC4_PreviousValue;
		}
		else
		{
			result = (TIM4_PERIOD - TIM4_IC4_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15) != 0)
		{
			// Input is not 0, transition from low to high
			TIM4_IC4_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM4_IC4_HighWidth = result;
			RCData.ch[3].PWMIN = (uint16_t)TIM4_IC4_HighWidth;
			RCData.ch[3].PWM_Good = 1;
			RCData.ch[3].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare1(TIM1, RCData.ch[3].PWMIN);
				RCData.ch[3].PWMOUT = RCData.ch[3].PWMIN;
			}
		}
		TIM4_IC4_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
		TIM_ClearFlag(TIM4, TIM_FLAG_CC4);
	}
}

void TIM8_CC_ISR_Handler(void)
{
	uint32_t dataTemp = 0;
	uint32_t result = 0;
	if((TIM8->SR & TIM_FLAG_CC1) != (u16)RESET)
	{
		// CC on channel 1
		dataTemp = TIM_GetCapture1(TIM8);
		// Calculate time
		// Check value
		if(dataTemp > TIM8_IC1_PreviousValue)
		{
			result = dataTemp - TIM8_IC1_PreviousValue;
		}
		else
		{
			result = (TIM8_PERIOD - TIM8_IC1_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) != 0)
		{
			// Input is not 0, transition from low to high
			TIM8_IC1_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM8_IC1_HighWidth = result;
			RCData.ch[4].PWMIN = (uint16_t)TIM8_IC1_HighWidth;
			RCData.ch[4].PWM_Good = 1;
			RCData.ch[4].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare4(TIM3, RCData.ch[4].PWMIN);
				RCData.ch[4].PWMOUT = RCData.ch[4].PWMIN;
			}
		}
		TIM8_IC1_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
		TIM_ClearFlag(TIM8, TIM_FLAG_CC1);
	}
	else if((TIM8->SR & TIM_FLAG_CC2) != (u16)RESET)
	{
		// CC on channel 2
		dataTemp = TIM_GetCapture2(TIM8);
		// Calculate time
		// Check value
		if(dataTemp > TIM8_IC2_PreviousValue)
		{
			result = dataTemp - TIM8_IC2_PreviousValue;
		}
		else
		{
			result = (TIM8_PERIOD - TIM8_IC2_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) != 0)
		{
			// Input is not 0, transition from low to high
			TIM8_IC2_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM8_IC2_HighWidth = result;
			RCData.ch[5].PWMIN = (uint16_t)TIM8_IC2_HighWidth;
			RCData.ch[5].PWM_Good = 1;
			RCData.ch[5].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare3(TIM3, RCData.ch[5].PWMIN);
				RCData.ch[5].PWMOUT = RCData.ch[5].PWMIN;
			}
		}
		TIM8_IC2_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
		TIM_ClearFlag(TIM8, TIM_FLAG_CC2);
	}

	else if((TIM8->SR & TIM_FLAG_CC3) != (u16)RESET)
	{
		// CC on channel 3
		dataTemp = TIM_GetCapture3(TIM8);
		// Calculate time
		// Check value
		if(dataTemp > TIM8_IC3_PreviousValue)
		{
			result = dataTemp - TIM8_IC3_PreviousValue;
		}
		else
		{
			result = (TIM8_PERIOD - TIM8_IC3_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) != 0)
		{
			// Input is not 0, transition from low to high
			TIM8_IC3_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM8_IC3_HighWidth = result;
			RCData.ch[6].PWMIN = (uint16_t)TIM8_IC3_HighWidth;
			RCData.ch[6].PWM_Good = 1;
			RCData.ch[6].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare2(TIM3, RCData.ch[6].PWMIN);
				RCData.ch[6].PWMOUT = RCData.ch[6].PWMIN;
			}
		}
		TIM8_IC3_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
		TIM_ClearFlag(TIM8, TIM_FLAG_CC3);
	}

	else if((TIM8->SR & TIM_FLAG_CC4) != (u16)RESET)
	{
		// CC on channel 4
		dataTemp = TIM_GetCapture4(TIM8);
		// Calculate time
		// Check value
		if(dataTemp > TIM8_IC4_PreviousValue)
		{
			result = dataTemp - TIM8_IC4_PreviousValue;
		}
		else
		{
			result = (TIM8_PERIOD - TIM8_IC4_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) != 0)
		{
			// Input is not 0, transition from low to high
			TIM8_IC4_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM8_IC4_HighWidth = result;
			RCData.ch[7].PWMIN = (uint16_t)TIM8_IC4_HighWidth;
			RCData.ch[7].PWM_Good = 1;
			RCData.ch[7].PWM_Timeout = 0;
			if(PWM_PASSTHROUGH)
			{
				TIM_SetCompare1(TIM3, RCData.ch[7].PWMIN);
				RCData.ch[7].PWMOUT = RCData.ch[7].PWMIN;
			}
		}
		TIM8_IC4_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
		TIM_ClearFlag(TIM8, TIM_FLAG_CC4);
	}
}

/**
  * @brief  This function handles Timer 9 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM9
  */
void TIM1_BRK_TIM9_ISR_Handler(void)
{
	// Received signal strength indicator
	uint32_t dataTemp = 0;
	uint32_t result = 0;
	float32_t strength = 0;
	if((TIM9->SR & TIM_FLAG_CC1) != (u16)RESET)
	{
		// CC on channel 1
		dataTemp = TIM_GetCapture1(TIM9);
		// Reset signal strength timeout
		signalStrengthCount = 0;
		// Calculate time
		// Check value
		if(dataTemp > TIM9_IC1_PreviousValue)
		{
			result = dataTemp - TIM9_IC1_PreviousValue;
		}
		else
		{
			result = (TIM9_PERIOD - TIM9_IC1_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) != 0)
		{
			// Input is not 0, transition from low to high
			TIM9_IC1_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM9_IC1_HighWidth = result;
			// Calculate DC
			// Calculate period
			strength = TIM9_IC1_HighWidth + TIM9_IC1_LowWidth;
			// Calculate DC
			strength = TIM9_IC1_LowWidth / strength;
			strength = strength * 100;
			RCData.RSSI = (uint16_t)strength;

		}
		TIM9_IC1_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
		TIM_ClearFlag(TIM9, TIM_FLAG_CC1);
	}
}


/**
  * @brief  This function handles Timer 14 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM14
  * 1 ms interval
  */
void TIM8_TRG_COM_TIM14_ISR_Handler(void)
{

	uint8_t retriesCount = 0;
	uint16_t ui16Temp;
	ErrorStatus error = SUCCESS;
	if((TIM14->SR & TIM_FLAG_Update) != (u16)RESET)
	{
		TIM_ClearFlag(TIM14, TIM_FLAG_Update);
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

		// Update system time
		systemTime++;
#ifndef USE_FREEMASTER
		// UART2 timeout
		UART_Timeout();
#endif
		// RS485 timing
		//RS485_Timing();

		// Check RC inputs for timeouts
		CheckRCInputTimeouts();

		// Signal strength count
		signalStrengthCount++;
		if(signalStrengthCount > SIGNALSTRENGTH_MAXTIME)
		{
			signalStrengthCount = SIGNALSTRENGTH_MAXTIME;
			// Check input polarity
			if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) != 0)
			{
				RCData.RSSI = 0;
			}
			else
			{
				RCData.RSSI = 100;
			}
		}
		//DEBUG_PIN_TOGGLE;
		if(PWMEN_OUT_ENABLE)
		{
			PWMEN_PIN_TOGGLE;
		}


		if(ADC_ENABLED)
		{
			//ADC_TriggerTimer++;
			//if(ADC_TriggerTimer >= 1)
			{
				//ADC_TriggerTimer = 0;
	        	ADC_SoftwareStartConv(ADC1);
	        	ADC_SoftwareStartConv(ADC2);
	        	ADC_SoftwareStartConv(ADC3);
			}
		}

		if(EXTSENS_INIT_DONE)
		{
			// Check I2C sensors
			I2C2_PollTimer++;
			if(I2C2_PollTimer >= I2C2_POLLTIME)
			{
				I2C2_PollTimer = I2C2_POLLTIME;
				if(!I2C2_WAITINGDATA && I2C2_INITDONE && MPU_COMM_ENABLED)
				{

					// Store start time
					ui32StartTime = getSystemTime();
					I2C2_PollTimer = 0;
					for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
					{
						// Mark time when data is requested
						sensorAcquisitionTime = getSystemTime();
						// Read from MPU, start at reg 59, read 25 bytes
						//error = masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 26);
						error = masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_sensorBufRX.buf, 26);
						if(error == SUCCESS)
						{
							break;
						}
						else
						{
							// Handle error
							I2C2_ResetInterface();
						}
					}
					// Store system time
					fusionData.deltaTime = systemTime - fusionData.dataTime;
					fusionData.dataTime = systemTime;
					// Update RS485
					//Refresh485();
				}
			}
		}


		// Call RS485 states
		//RS485_States_Master();

		// LED counter
		LED_ToggleCount++;
		if(LED_ToggleCount >= 100)
		{
			LED_ToggleCount = 0;
			// Send out data
#ifndef USE_FREEMASTER
			COMM_SEND_DATA = 1;
#endif
			// Event every second

			// Write log
			/*
			if(SD_WRITE_LOG && SCR2_LOGOPEN)
			{
				write_toLog();
			}
			*/

			// Toggle LED
			// Check GPS data valid - MODBUSREG[21] bit 15
			/*
			if(MODBUSReg[21] & 32768)
			{
				LED_OK_ON;
			}
			else
			{
				LED_OK_TOGGLE;
			}*/
		}
		// Call fatfs timer
		SD_TimerCount++;
		if(SD_TimerCount >= 10)
		{
			SD_TimerCount = 0;
			disk_timerproc();
		}
		// Call sensor timer
		sensorInterruptTimer();

		// Send status
		if(SYSTEM_RUNNING)
		{
			ui32CANTime++;
			ui32SendAHRSDataTime++;

			if(100 < ui32SendAHRSDataTime)
			{
				ui32SendAHRSDataTime = 0;
				CAN_SendOrientation();
				CAN_SendOrientationPID();
			}
			else if(500 < ui32CANTime)
			{
				ui32CANTime = 0;
				CAN_SendNodeStatus();
			}
			ui16SendMavlinkHeartbeetTime++;
			//ui16SendMavlinkBatteryStatus++;
			if(1000 < ui16SendMavlinkHeartbeetTime)
			{
				if(0 == mavlinkSendBusy)
				{
					ui16Temp = mavlink_msg_heartbeat_pack(1, 1, &mavlinkMessageDataTX, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					ui16SendMavlinkHeartbeetTime = 0;
				}
				else
				{
					ui16SendMavlinkHeartbeetTime --;
				}
			}


			ui16SendMavlinkBatteryStatus++;
			if(1500 < ui16SendMavlinkBatteryStatus)
			{
				if(0 == mavlinkSendBusy)
				{
					ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 1, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 25, &FCFlightData.batMon.ui6MavLinkVoltage, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1, -1, 	MAV_BATTERY_CHARGE_STATE_OK);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					ui16SendMavlinkBatteryStatus = 0;
				}
				else
				{
					ui16SendMavlinkBatteryStatus--;
				}
			}
		}
	}
}

/**
  * @brief  This function handles TIM1 capture compare interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_CC_ISR_Handler(void)
{
	//check what triggered event
	if((TIM1->SR & TIM_FLAG_CC1) != (u16)RESET)
	{
		//read IC1 value
		TIM1CaptureValue1 = TIM_GetCapture1(TIM1);
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
	}
	if((TIM1->SR & TIM_FLAG_CC2) != (u16)RESET)
	{
		//read IC2 value
		TIM1CaptureValue2 = TIM_GetCapture2(TIM1);
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
	}
	if((TIM1->SR & TIM_FLAG_CC3) != (u16)RESET)
	{
		//read IC3 value
		TIM1CaptureValue3 = TIM_GetCapture3(TIM1);
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
	}
	if((TIM1->SR & TIM_FLAG_CC4) != (u16)RESET)
	{
		//read IC4 value
		TIM1CaptureValue4 = TIM_GetCapture4(TIM1);
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
		TIM_ClearFlag(TIM1, TIM_FLAG_CC4);
	}
}

/**
  * @brief  This function handles CAN1 interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_RX0_ISR_Handler()
{
	CanRxMsg RecMessage;
	// Get how many messages are in fifo
	uint8_t ui8MessagesPending = CAN_MessagePending(CAN1, CAN_FIFO0);
	while(0 != ui8MessagesPending)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RecMessage);
		ProcessCANMessage(&RecMessage);
		// Release FIFO
		//CAN_FIFORelease(CAN1, CAN_FIFO0);
		// Check buffer
		ui8MessagesPending = CAN_MessagePending(CAN1, CAN_FIFO0);
	}
}

void CAN1_RX1_ISR_Handler()
{
	CanRxMsg RecMessage;
	// Get how many messages are in fifo
	uint8_t ui8MessagesPending = CAN_MessagePending(CAN1, CAN_FIFO1);
	while(0 != ui8MessagesPending)
	{
		CAN_Receive(CAN1, CAN_FIFO1, &RecMessage);
		ProcessCANMessage(&RecMessage);
		// Release FIFO
		//CAN_FIFORelease(CAN1, CAN_FIFO1);
		// Check buffer
		ui8MessagesPending = CAN_MessagePending(CAN1, CAN_FIFO1);
	}
}

void CAN1_TX0_ISR_Handler()
{
	// Check CAN message buffer
	if(CANData.ui16CANTxMsgBufRead != CANData.ui16CANTxMsgBufStore)
	{
		// Messages in buffer. Transmit next message
		uint8_t txResult = CAN_Transmit(CAN1, &CANData.CANTxMsgBuf[CANData.ui16CANTxMsgBufRead]);
		if(CAN_TxStatus_NoMailBox != txResult)
		{
			// Msg stored in tx mailbox, remove from queue
			CANData.ui16CANTxMsgBufRead++;
			CANData.ui16CANTxMsgBufRead = CANData.ui16CANTxMsgBufRead & 0x1f;
		}
	}
	else
	{
		// No more messages in buffer, disable TX interrupt
		CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
	}
	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
}

/**
  * @brief  This function handles CAN1 SCE interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_SCE_ISR_Handler(void)
{
	CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_ISR_Handler(void)
{
	int iData = 0;
	uint8_t msgStatus = 0;
	if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)	//if new data in
	{
		iData = USART_ReceiveData(USART1);
		//GPS_ReceiveProcess((uint8_t)iData, getSystemTime());
		msgStatus = mavlink_parse_char(1, (uint8_t)iData, &mavlinkMessageDataRX, &mavlinkStatusData);
		if(0 != msgStatus)
		{
			msgStatus = 0;
		}
	}

	if((USART1->SR & USART_FLAG_TC) != (u16)RESET)	//if transfer complete
	{
		USART1->SR = USART1->SR & !USART_FLAG_TC;
	}
}


/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_ISR_Handler(void)
{
#ifndef USE_FREEMASTER
	if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)	//if new data in
	{

		RB_push(&RB_USART2, (uint8_t)(USART2->DR & (uint16_t)0x01FF));
		//UART_RcvData((uint8_t)(USART2->DR & (uint16_t)0x01FF));
	}
	if((USART2->SR & USART_FLAG_TC) != (u16)RESET)	//if transfer complete
	{
		// Clear TC
		USART2->SR = USART2->SR & !USART_FLAG_TC;
		// Disable TC interrupt
		USART_ITConfig(USART2, USART_IT_TC, DISABLE);
		// Mark not transferring
		UART2_Transferring = 0;
		// Check to send data
		UART_SendBuffer();
	}
#endif
}

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_ISR_Handler(void)
{
	int iData = 0;
	if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)	//if new data in
	{
		iData = USART_ReceiveData(USART3);
		//GPS_ReceiveProcess((uint8_t)iData, getSystemTime());
		ubx_parser((uint8_t)iData);
	}

	if((USART3->SR & USART_FLAG_TC) != (u16)RESET)	//if transfer complete
	{
		USART3->SR = USART3->SR & !USART_FLAG_TC;
	}
}

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_ISR_Handler(void)
{
	uint32_t dtCalc = 0;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{

		dtCalc = getSystemTime();
		fusionData.sensorInterruptDeltaTime = dtCalc - fusionData.sensorInterruptTime;
		fusionData.sensorInterruptTime = dtCalc;

		/*
		// Get data from sensors
		if(!I2C2_WAITINGDATA && I2C2_INITDONE)
		{
			I2C2_PollTimer = 0;
			for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
			{
				// Mark time when data is requested
				sensorAcquisitionTime = getSystemTime();
				// Read from MPU, start at reg 59, read 25 bytes
				//error = masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 26);
				error = masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_sensorBufRX.buf, 26);
				if(error == SUCCESS)
				{
					break;
				}
				else
				{
					// Handle error
					I2C2_ResetInterface();
				}
			}
		}

		// Store system time
		fusionData.deltaTime = systemTime - fusionData.dataTime;
		fusionData.dataTime = systemTime;
		*/
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/**
  * @brief  This function handles FPU interrupt request.
  * @param  None
  * @retval None
  */
void FPU_ISR_Handler(void)
{
	FPU_EXCEPTION = 1;
	// Clear FPU flags in flagReg1
	CLEAR_FPU_EXCEPTIONS;
	// Clear exception flag(s)
	register uint32_t fpscr_val = 0;
	register uint32_t fpccr_val = 0;
	register uint32_t reg_val = 0;

	uint32_t * FPCCR_REG;

	FPCCR_REG = (uint32_t *)0xE000EF34;

	fpccr_val = *FPCCR_REG;

	uint32_t bits = fpccr_val & 0XC0000000;

	bits = bits >> 30;

	if(0 == bits)
	{
		reg_val = 1;
		fpscr_val = __get_FPSCR();
		//{ check exception flags }
		fpscr_val &= (uint32_t)~0x8F; // Clear all exception flags
		__set_FPSCR(fpscr_val);
	}
	else if(2 == bits)
	{

	}
	else
	{
		reg_val = __get_FPSCR(); //dummy access
		fpscr_val=*(__IO uint32_t*)(FPU->FPCAR +0x40);
		//{ check exception flags }
		fpscr_val &= (uint32_t)~0x8F ; // Clear all exception flags
		*(__IO uint32_t*)(FPU->FPCAR +0x40)=fpscr_val;
		__DMB() ;
	}
}


