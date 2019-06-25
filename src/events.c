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

		//ui16MavlinkBatteryVoltages
		fTemp = FCFlightData.batMon.fUBat * 1000.0f;

		FCFlightData.batMon.ui16MavlinkTotalBatteryVoltage = (uint16_t)fTemp;

		fTemp = FCFlightData.batMon.fUBat / ((float32_t)FCFlightData.batMon.ui16NumCells);

		fTemp = FCFlightData.batMon.ui16MavlinkBatteryVoltages[0] = (uint16_t)fTemp;
		FCFlightData.batMon.ui16MavlinkBatteryVoltages[1] = FCFlightData.batMon.ui16MavlinkBatteryVoltages[0];
		FCFlightData.batMon.ui16MavlinkBatteryVoltages[2] = FCFlightData.batMon.ui16MavlinkBatteryVoltages[0];
		FCFlightData.batMon.ui16MavlinkBatteryVoltages[3] = FCFlightData.batMon.ui16MavlinkBatteryVoltages[0];


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

		fTemp = FCFlightData.batMon.fIBat * 100.0f;

		FCFlightData.batMon.i16MavLinkCurrent = (int16_t)(fTemp);
		// Clear flag
		ADC_ClearFlag(ADC3, ADC_FLAG_EOC);
		// Clear interrupt
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);

		// To get mA, mul I * 1000; time is 1 ms, so div by 1000.
		FCFlightData.batMon.fmAsUsed += FCFlightData.batMon.fIBat;
		FCFlightData.batMon.fmAhUsed = FCFlightData.batMon.fmAsUsed / 3600.0f;

		FCFlightData.batMon.i32MavLinkCurrentConsumed = (int32_t)(FCFlightData.batMon.fmAhUsed);
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
  * @brief  This function handles DMA1 stream0 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 0 - SPI3 RX
  */
void DMA1_Stream0_ISR_Handler(void)
{
	Vectorf vfTempVector;
	//float32_t fTemp;
	// Disable interrupts
	DMA_ITConfig(DMA_SPI3_RX_STREAM, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, DISABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_SPI3_RX_STREAM, DISABLE);

	/* Wait until SPI_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_SPI3_RX_STREAM)!= DISABLE)
	{}
	/* Disable SPI DMA request */
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, DISABLE);
	DMA_ClearITPendingBit(DMA_SPI3_RX_STREAM, DMA_IT_TC);
	// Set CS = 1
	switch(SPI_SensorBuf->ui8Device)
	{
		case GYRO_DEV_CS:
		{
			A_G_CS_1;
			// Update gyro data
			// Convert to rad/sec

			fusionData._gyro.vectorRaw.x = (float32_t)SPI_SensorBufGyro.DATA.GYRO.i16XOut * fusionData._gyro.gyroRateXP * GYRO_DEG_TO_RAD;
			fusionData._gyro.vector.x = fusionData._gyro.vectorRaw.x - fusionData._gyroErrorPID.x.s;

			fusionData._gyro.vectorRaw.y = (float32_t)SPI_SensorBufGyro.DATA.GYRO.i16YOut * fusionData._gyro.gyroRateYP * GYRO_DEG_TO_RAD;
			fusionData._gyro.vector.y = fusionData._gyro.vectorRaw.y - fusionData._gyroErrorPID.y.s;

			fusionData._gyro.vectorRaw.z = (float32_t)SPI_SensorBufGyro.DATA.GYRO.i16ZOut * fusionData._gyro.gyroRateZP * GYRO_DEG_TO_RAD;
			fusionData._gyro.vector.z = fusionData._gyro.vectorRaw.z - fusionData._gyroErrorPID.z.s;


			// Time from last sample
			fusionData._gyro.fDeltaTime = (float32_t)SPI_SensorBufGyro.ui32InterruptDeltaTime * fusionData.PARAMETERS.systimeToSeconds;

			// Update DCM
			fusion_dataUpdate(&fusionData, fusionData._gyro.fDeltaTime);

			storeAHRSAngles(&fusionData);

			// Update flight data
			FCFlightData.ORIENTATION.f32Roll = fusionData.ROLLPITCHYAW.roll - FCFlightData.ORIENTATION.f32RollOffset;
			FCFlightData.ORIENTATION.f32Pitch = fusionData.ROLLPITCHYAW.pitch - FCFlightData.ORIENTATION.f32PitchOffset;
			FCFlightData.ORIENTATION.f32Yaw = fusionData.ROLLPITCHYAW.yaw - FCFlightData.ORIENTATION.f32YawOffset;
			FCFlightData.ORIENTATION.f32Altitude = fusionData._altimeter.altitude;

			break;
		}
		case ACC_DEV_CS:
		{
			A_G_CS_1;

			// Update accelerometer reading

			// Store previous vector
			vectorf_copy(&fusionData._accelerometer.vector, &fusionData._accelerometer.vector_m);
			// Store raw data, multiply with - to get gravity - acceleration
			fusionData._accelerometer.vectorRaw.x = (float32_t)SPI_SensorBufAcc.DATA.ACC.i16XOut * -fusionData._accelerometer.accRate;
			fusionData._accelerometer.vectorRaw.y = (float32_t)SPI_SensorBufAcc.DATA.ACC.i16YOut * -fusionData._accelerometer.accRate;
			fusionData._accelerometer.vectorRaw.z = (float32_t)SPI_SensorBufAcc.DATA.ACC.i16ZOut * -fusionData._accelerometer.accRate;



			// Filter
			fusionData._accelerometer.filterAccum.x += fusionData._accelerometer.vectorRaw.x;
			fusionData._accelerometer.vectorKFiltered.x = fusionData._accelerometer.filterAccum.x / 5.0f;
			fusionData._accelerometer.filterAccum.x -= fusionData._accelerometer.vectorKFiltered.x;

			fusionData._accelerometer.filterAccum.y += fusionData._accelerometer.vectorRaw.y;
			fusionData._accelerometer.vectorKFiltered.y = fusionData._accelerometer.filterAccum.y / 5.0f;
			fusionData._accelerometer.filterAccum.y -= fusionData._accelerometer.vectorKFiltered.y;

			fusionData._accelerometer.filterAccum.z += fusionData._accelerometer.vectorRaw.z;
			fusionData._accelerometer.vectorKFiltered.z = fusionData._accelerometer.filterAccum.z / 5.0f;
			fusionData._accelerometer.filterAccum.z -= fusionData._accelerometer.vectorKFiltered.z;

			// Remove offset
			vectorf_add(&fusionData._accelerometer.vectorKFiltered, &fusionData._accelerometer.offsets, &fusionData._accelerometer.vector);

			// Use gain correction on outputs?
			fusionData._accelerometer.vector.x *= fusionData._accelerometer.gains.x;
			fusionData._accelerometer.vector.y *= fusionData._accelerometer.gains.y;
			fusionData._accelerometer.vector.z *= fusionData._accelerometer.gains.z;


			// Also store normalized vector
			vectorf_normalizeAToB(&fusionData._accelerometer.vector, &fusionData._accelerometer.vectorNormalized);
			// Store vector length
			fusionData._accelerometer.vectorNorm = vectorf_getNorm(&fusionData._accelerometer.vector);
			// Store time information
			fusionData._accelerometer.deltaTime = (float32_t)SPI_SensorBufAcc.ui32InterruptDeltaTime * fusionData.PARAMETERS.systimeToSeconds;


			//*****************************
			// Use accelerometer to determine orientation -> acceleration history is 0
			// Calculate delta acc
			// C = A - B
			vectorf_substract(&fusionData._accelerometer.vectorHistory[fusionData._accelerometer.ui8CurrentHistory], &fusionData._accelerometer.vector, &vfTempVector);

			// Increase address
			fusionData._accelerometer.ui8CurrentHistory++;
			fusionData._accelerometer.ui8CurrentHistory &= 0x07;

			// Calculate new vector average
			vectorf_substract(&fusionData._accelerometer.vectorAverage, &fusionData._accelerometer.vectorHistory[fusionData._accelerometer.ui8CurrentHistory], &fusionData._accelerometer.vectorAverage);
			vectorf_add(&fusionData._accelerometer.vectorAverage, &fusionData._accelerometer.vector, &fusionData._accelerometer.vectorAverage);
			// Calculate new acceleration average
			vectorf_substract(&fusionData._accelerometer.accTotal, &fusionData._accelerometer.accHistory[fusionData._accelerometer.ui8CurrentHistory], &fusionData._accelerometer.accTotal);
			vectorf_add(&fusionData._accelerometer.accTotal, &vfTempVector, &fusionData._accelerometer.accTotal);
			// Store to history
			vectorf_copy(&fusionData._accelerometer.vector, &fusionData._accelerometer.vectorHistory[fusionData._accelerometer.ui8CurrentHistory]);
			vectorf_copy( &vfTempVector, &fusionData._accelerometer.accTotal);

			fusionData._accelerometer.accTotalNorm = vectorf_getNorm(&fusionData._accelerometer.accTotal);


			// Store DCM estimation of acc reading
			vectorf_copy(&fusionData._fusion_DCM.c, &fusionData._gyroError.DCMDown);

			fusionData._gyroError.DCMDown.x = -fusionData._gyroError.DCMDown.x;
			fusionData._gyroError.DCMDown.y = -fusionData._gyroError.DCMDown.y;
			fusionData._gyroError.DCMDown.z = -fusionData._gyroError.DCMDown.z;

			// Transform gravity to earth coordinates
			matrix3_transposeVectorMultiply(&fusionData._fusion_DCM, &fusionData._accelerometer.vector, &fusionData._accelerometer.vector_earth);

			vectorf_normalizeAToB(&fusionData._accelerometer.vector_earth, &fusionData._accelerometer.vectorNorm_earth);


			// Calculate gravity error
			if((0.98f < fusionData._accelerometer.vectorNorm)&&(1.02 > fusionData._accelerometer.vectorNorm))
			{
				vfTempVector.x = 0;
				vfTempVector.y = 0;
				vfTempVector.z = 1;
				vectorf_crossProduct(&vfTempVector, &fusionData._accelerometer.vectorNorm_earth, &fusionData._gyroError.AccError);

				// Transform to plane coordinates
				matrix3_vectorMultiply(&fusionData._fusion_DCM, &fusionData._gyroError.AccError, &fusionData._gyroError.AccError);

				vectorf_scalarProduct(&fusionData._gyroError.AccError, 0.1f, &fusionData._gyroError.AccError);
			}
			else
			{
				fusionData._gyroError.AccError.x = 0.0f;
				fusionData._gyroError.AccError.y = 0.0f;
				fusionData._gyroError.AccError.z = 0.0f;
			}



			// Use accelerometer data to determine no rotation -> gyro should be 0
			vectorf_crossProduct(&fusionData._accelerometer.vector, &fusionData._accelerometer.vector_m, &vfTempVector);
			fusionData._accelerometer.f32VectorAvgDiff += vectorf_getNorm(&vfTempVector);
			fusionData._accelerometer.f32VectorAvgDiff *= 0.5f;

			// If average acc change is below limit (= no rotation), adjust gyro offset towards zero.
			if(fusionData._accelerometer.f32MinChange > fusionData._accelerometer.f32VectorAvgDiff)
			{
				// Gyro error is current gyro reading * some value
				vectorf_scalarProduct(&fusionData._gyro.vector, fusionData._gyro.errorScale, &fusionData._gyroError.GyroError);
				math_PID3(&fusionData._gyroError.GyroError, fusionData._gyro.fDeltaTime, &fusionData._gyroErrorPID);
			}

			break;
		}
		case MAG_DEV_CS:
		{
			MAG_CS_1;

			// Update current reading
			fusionData._mag.vectorRaw.x = (float32_t)SPI_SensorBufMag.DATA.MAG.i16XOut * fusionData._mag.magRate;
			fusionData._mag.vectorRaw.y = (float32_t)SPI_SensorBufMag.DATA.MAG.i16YOut * fusionData._mag.magRate;
			fusionData._mag.vectorRaw.z = (float32_t)SPI_SensorBufMag.DATA.MAG.i16ZOut * fusionData._mag.magRate;

			//Filter
			fusionData._mag.vectorKFiltered.x = Kalman_Update(&fusionData._mag.kFilter.X, fusionData._mag.vectorRaw.x);
			fusionData._mag.vectorKFiltered.y = Kalman_Update(&fusionData._mag.kFilter.Y, fusionData._mag.vectorRaw.y);
			fusionData._mag.vectorKFiltered.z = Kalman_Update(&fusionData._mag.kFilter.Z, fusionData._mag.vectorRaw.z);

			// Use soft iron matrix to correct mag reading
			// Remove offset
			vectorf_substract(&fusionData._mag.vectorKFiltered, &fusionData._mag.offset, &fusionData._mag.currentMagReading);
			// Multiply with soft iron matrix to correct distortions
			matrix3_vectorMultiply(&fusionData._mag.softIron, &fusionData._mag.currentMagReading, &fusionData._mag.vector);
			// Normalize mag vector

			break;
		}
		case BARO_DEV_CS:
		{
			BARO_CS_1;
			SPI_SensorBufBaro.DATA.buf[9] = SPI_SensorBufBaro.DATA.buf[8];
			SPI_SensorBufBaro.DATA.buf[8] = SPI_SensorBufBaro.DATA.buf[7];
			if(0x80 & SPI_SensorBufBaro.DATA.buf[6])
			{
				SPI_SensorBufBaro.DATA.buf[7] = 0xff;
			}
			else
			{
				SPI_SensorBufBaro.DATA.buf[7] = 0;
			}

			// Process baro data
			break;
		}
	}
	// Check if more data to be read
	Sensor_SPICommProcess();
}

/**
  * @brief  This function handles DMA1 stream0 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 0 - SPI3 TX
  */
void DMA1_Stream5_ISR_Handler(void)
{
	// Disable interrupts
	DMA_ITConfig(DMA_SPI3_TX_STREAM, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, DISABLE);


	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_SPI3_TX_STREAM, DISABLE);

	/* Wait until SPI_DMA_STREAM_RX disabled or time out */
	while (DMA_GetCmdStatus(DMA_SPI3_TX_STREAM)!= DISABLE)
	{}
	/* Disable SPI DMA request */
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);
	DMA_ClearITPendingBit(DMA_SPI3_TX_STREAM, DMA_IT_TC);
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
  * @brief  This function handles Timer 3 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM3
  */
void TIM3_ISR_Handler(void)
{
	uint32_t dataTemp = 0;
	uint32_t result = 0;
	if((TIM3->SR & TIM_FLAG_CC1) != (u16)RESET)
	{
		// CC on channel 1
		dataTemp = TIM_GetCapture1(TIM3);
		// Calculate time
		// Check value
		if(dataTemp > TIM3_IC1_PreviousValue)
		{
			result = dataTemp - TIM3_IC1_PreviousValue;
		}
		else
		{
			result = (TIM3_PERIOD - TIM3_IC1_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) != 0)
		{
			// Input is not 0, transition from low to high
			TIM3_IC1_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM3_IC1_HighWidth = result;
			RCData.ch[10].PWMIN = (uint16_t)TIM3_IC1_HighWidth;
			RCData.ch[10].PWM_Good = 1;
			RCData.ch[10].PWM_Timeout = 0;
		}
		TIM3_IC1_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
	}

	else if((TIM3->SR & TIM_FLAG_CC2) != (u16)RESET)
	{
		// CC on channel 2
		dataTemp = TIM_GetCapture2(TIM3);
		// Calculate time
		// Check value
		if(dataTemp > TIM3_IC2_PreviousValue)
		{
			result = dataTemp - TIM3_IC2_PreviousValue;
		}
		else
		{
			result = (TIM3_PERIOD - TIM3_IC2_PreviousValue) + dataTemp;
		}
		// Check input polarity
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) != 0)
		{
			// Input is not 0, transition from low to high
			TIM3_IC2_LowWidth = result;
		}
		else
		{
			// Else transition from high to low
			TIM3_IC2_HighWidth = result;
			RCData.ch[11].PWMIN = (uint16_t)TIM3_IC2_HighWidth;
			RCData.ch[11].PWM_Good = 1;
			RCData.ch[11].PWM_Timeout = 0;
		}
		TIM3_IC2_PreviousValue = dataTemp;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		TIM_ClearFlag(TIM3, TIM_FLAG_CC2);
	}
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

	//uint8_t retriesCount = 0;
	//uint16_t ui16Temp;

	T32BITVARS T32Temp;

	//ErrorStatus error = SUCCESS;
	if((TIM14->SR & TIM_FLAG_Update) != (u16)RESET)
	{
		TIM_ClearFlag(TIM14, TIM_FLAG_Update);
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);

		// Call freemaster recorder
		FMSTR_Recorder();

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

		if(ADC_ENABLED)
		{
			ADC_SoftwareStartConv(ADC1);
	        ADC_SoftwareStartConv(ADC2);
	        ADC_SoftwareStartConv(ADC3);
		}


		Sensor_SPICommProcess();

#ifndef SPI_SENSOR_TESTING

		// Check sensors
		SPI_SensorBufAcc.ui16WatchdogCounter++;
		SPI_SensorBufGyro.ui16WatchdogCounter++;
		SPI_SensorBufMag.ui16WatchdogCounter++;
		SPI_SensorBufBaro.ui16WatchdogCounter++;
		if(10 < SPI_SensorBufAcc.ui16WatchdogCounter)
		{
			SPI_SensorBufAcc.ui16WatchdogCounter = 0;
			// Trigger update
			SPI_SensorBufAcc.ui32InterruptDeltaTime = READ_SYS_TIME - SPI_SensorBufAcc.ui32InterruptTime;
			SPI_SensorBufAcc.ui32InterruptTime = READ_SYS_TIME;
			SPI_SensorBufAcc.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, ACC_GET_DATA);
		}
		if(10 < SPI_SensorBufGyro.ui16WatchdogCounter)
		{
			SPI_SensorBufGyro.ui16WatchdogCounter = 0;
			// Trigger update
			SPI_SensorBufGyro.ui32InterruptDeltaTime = READ_SYS_TIME - SPI_SensorBufGyro.ui32InterruptTime;
			SPI_SensorBufGyro.ui32InterruptTime = READ_SYS_TIME;
			SPI_SensorBufGyro.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, GYRO_GET_DATA);
		}
		if(15 < SPI_SensorBufMag.ui16WatchdogCounter)
		{
			SPI_SensorBufMag.ui16WatchdogCounter = 0;
			// Trigger update
			SPI_SensorBufMag.ui32InterruptDeltaTime = READ_SYS_TIME - SPI_SensorBufMag.ui32InterruptTime;
			SPI_SensorBufMag.ui32InterruptTime = READ_SYS_TIME;
			SPI_SensorBufMag.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, MAG_GET_DATA);
		}
		if(50 < SPI_SensorBufBaro.ui16WatchdogCounter)
		{
			SPI_SensorBufBaro.ui16WatchdogCounter = 0;
			// Trigger update
			SPI_SensorBufBaro.ui32InterruptDeltaTime = READ_SYS_TIME - SPI_SensorBufBaro.ui32InterruptTime;
			SPI_SensorBufBaro.ui32InterruptTime = READ_SYS_TIME;
			SPI_SensorBufBaro.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, BARO_GET_DATA);
		}
#endif

		// LED counter
		LED_ToggleCount++;
		if(LED_ToggleCount >= 100)
		{
			LED_ToggleCount = 0;
			LED_OK_TOGGLE;
			// Send out data
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

		ui8MagDataTransferTime++;
		if(50 < ui8MagDataTransferTime)
		{
			ui8MagDataTransferTime = 0;
			if(UART4_SEND_VECTOR)
			{
				ui8MagDataTransfer[0] = 0xfa;

				/*
				// Raw value
				T32Temp.f = fusionData._mag.vectorKFiltered.x;
				ui8MagDataTransfer[1] = T32Temp.ui8[0];
				ui8MagDataTransfer[2] = T32Temp.ui8[1];
				ui8MagDataTransfer[3] = T32Temp.ui8[2];
				ui8MagDataTransfer[4] = T32Temp.ui8[3];

				T32Temp.f = fusionData._mag.vectorKFiltered.y;
				ui8MagDataTransfer[5] = T32Temp.ui8[0];
				ui8MagDataTransfer[6] = T32Temp.ui8[1];
				ui8MagDataTransfer[7] = T32Temp.ui8[2];
				ui8MagDataTransfer[8] = T32Temp.ui8[3];

				T32Temp.f = fusionData._mag.vectorKFiltered.z;
				ui8MagDataTransfer[9] = T32Temp.ui8[0];
				ui8MagDataTransfer[10] = T32Temp.ui8[1];
				ui8MagDataTransfer[11] = T32Temp.ui8[2];
				ui8MagDataTransfer[12] = T32Temp.ui8[3];
				*/

				// Adjusted value
				T32Temp.f = fusionData._mag.vector.x;
				ui8MagDataTransfer[1] = T32Temp.ui8[0];
				ui8MagDataTransfer[2] = T32Temp.ui8[1];
				ui8MagDataTransfer[3] = T32Temp.ui8[2];
				ui8MagDataTransfer[4] = T32Temp.ui8[3];

				T32Temp.f = fusionData._mag.vector.y;
				ui8MagDataTransfer[5] = T32Temp.ui8[0];
				ui8MagDataTransfer[6] = T32Temp.ui8[1];
				ui8MagDataTransfer[7] = T32Temp.ui8[2];
				ui8MagDataTransfer[8] = T32Temp.ui8[3];

				T32Temp.f = fusionData._mag.vector.z;
				ui8MagDataTransfer[9] = T32Temp.ui8[0];
				ui8MagDataTransfer[10] = T32Temp.ui8[1];
				ui8MagDataTransfer[11] = T32Temp.ui8[2];
				ui8MagDataTransfer[12] = T32Temp.ui8[3];


				ui8ui8MagDataTransferCount = 0;
				// Clear interrupt
				UART4->SR = UART4->SR & !USART_FLAG_TC;
				// Enable TX empty interrupt
				USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
				// Send data
				//USART_SendData(UART4, 0xfa);
			}
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

		// GPS timeout
		ubx_timeout();

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

			ui16SendMavlink10Hz++;
			ui16SendMavlink5Hz++;
			ui16SendMavlink2Hz++;
			ui16SendMavlink1Hz++;

			if(1000 < ui16SendMavlink1Hz)
			{
				if(0 == mavlinkSendBusy)
				{
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_HEARTBEAT);
					ui16SendMavlink1Hz = 0;
				}
				else
				{
					ui16SendMavlink1Hz --;
				}
			}
			if(500 < ui16SendMavlink2Hz)
			{
				if(0 == mavlinkSendBusy)
				{
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_SYS_STATUS);
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_BATTERY_STATUS);
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_GPS_RAW_INT);
					ui16SendMavlink2Hz = 0;
				}
				else
				{
					ui16SendMavlink2Hz--;
				}
			}
			if(200 < ui16SendMavlink5Hz)
			{
				if(0 == mavlinkSendBusy)
				{
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_VFR_HUD);
					ui16SendMavlink5Hz = 0;
				}
				else
				{
					ui16SendMavlink5Hz--;
				}
			}
			if(100 < ui16SendMavlink10Hz)
			{
				if(0 == mavlinkSendBusy)
				{
					RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_ATTITUDE);
					ui16SendMavlink10Hz = 0;
				}
				else
				{
					ui16SendMavlink10Hz--;
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
		CANData.ui32CANRXMessages++;
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
	CANData.ui32CANTXMessages++;
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
	mavlink_command_long_t mavCommand;
	uint8_t msgStatus = 0;
	uint32_t ui32Temp = 0;
	if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)	//if new data in
	{
		iData = USART_ReceiveData(USART1);
		//GPS_ReceiveProcess((uint8_t)iData, getSystemTime());
		msgStatus = mavlink_parse_char(1, (uint8_t)iData, &mavlinkMessageDataRX, &mavlinkStatusData);
		if(0 != msgStatus)
		{
			switch(mavlinkMessageDataRX.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					break;
				}
				case MAVLINK_MSG_ID_SYSTEM_TIME:
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				{
					RB32_push(&rb32MavlinkTXQueue, mavlinkMessageDataRX.msgid);
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					mavlink_msg_command_long_decode(&mavlinkMessageDataRX, &mavCommand);
					i32CurrentMavlinkCommand = (int32_t)mavCommand.command;
					switch(mavCommand.command)
					{
						case MAV_CMD_REQUEST_PROTOCOL_VERSION:
						{
							ui32Temp = (uint32_t)mavCommand.param1;
							switch(ui32Temp)
							{
								case 1:
								{
									// V2
									//RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_PROTOCOL_VERSION);
									break;
								}
								default:
								{
									break;
								}
							}
							break;
						}
						case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
						{
							ui32Temp = (uint32_t)mavCommand.param1;
							switch(ui32Temp)
							{
								case 1:
								{
									RB32_push(&rb32MavlinkTXQueue, MAVLINK_MSG_ID_AUTOPILOT_VERSION);
									break;
								}
								default:
								{
									break;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
					break;
				}
				default:
				{

					break;
				}
			}



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
  * @brief  This function handles UART4 interrupt request.
  * @param  None
  * @retval None
  */
void UART4_ISR_Handler(void)
{
	int iData = 0;

	if ((UART4->SR & USART_FLAG_RXNE) != (u16)RESET)	//if new data in
	{
		iData = USART_ReceiveData(UART4);
		if(0xaa == iData)
		{
			UART4_SEND_VECTOR = 1;
		}
		else
		{
			UART4_SEND_VECTOR = 0;
		}
	}

	if((UART4->SR & USART_FLAG_TXE) != (u16)RESET)	//if transfer complete
	{
		UART4->SR = UART4->SR & !USART_FLAG_TC;


		if(13 > ui8ui8MagDataTransferCount)
		{
			// Send data
			USART_SendData(UART4, ui8MagDataTransfer[ui8ui8MagDataTransferCount]);
			ui8ui8MagDataTransferCount++;
		}
		else
		{
			USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
		}
	}
}

/**
  * @brief  This function handles interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_ISR_Handler(void)
{
	uint32_t dtCalc = 0;
	// Store time of interrupt
	//uint32_t ui32InterruptTime = READ_SYS_TIME;

	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{

		dtCalc = getSystemTime();
		fusionData.sensorInterruptDeltaTime = dtCalc - fusionData.sensorInterruptTime;
		fusionData.sensorInterruptTime = dtCalc;



		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/**
  * @brief  This function handles EXTI3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_ISR_Handler(void)
{
	// Store time of interrupt
	uint32_t ui32InterruptTime = READ_SYS_TIME;

	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		/* Clear the EXTI line 3 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);
		// A3, I_D_INT_BARO_DRDY
		// C3, I_D_INT_M
		// Check lines A3/C3
#ifndef SPI_SENSOR_TESTING
		if(GPIOA->IDR & GPIO_Pin_3)
		{
			// Baro data ready nterrupt
			SPI_SensorBufBaro.ui32InterruptDeltaTime = ui32InterruptTime - SPI_SensorBufBaro.ui32InterruptTime;
			SPI_SensorBufBaro.ui32InterruptTime = ui32InterruptTime;
			SPI_SensorBufBaro.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, BARO_GET_DATA);
		}
#endif
	}
}

/**
  * @brief  This function handles EXTI4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_ISR_Handler(void)
{
	// Store time of interrupt
	uint32_t ui32InterruptTime = READ_SYS_TIME;

	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		/* Clear the EXTI line 4 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line4);
		// E4, I_D_MAG_DRDY
		// Check lines E4
#ifndef SPI_SENSOR_TESTING
		if(GPIOE->IDR & GPIO_Pin_4)
		{
			// Baro data ready nterrupt
			SPI_SensorBufMag.ui32InterruptDeltaTime = ui32InterruptTime - SPI_SensorBufMag.ui32InterruptTime;
			SPI_SensorBufMag.ui32InterruptTime = ui32InterruptTime;
			SPI_SensorBufMag.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, MAG_GET_DATA);
		}
#endif
	}
}

/**
  * @brief  This function handles EXTI14/15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_ISRHandler(void)
{
	// Store time of interrupt
	uint32_t ui32InterruptTime = READ_SYS_TIME;

	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		/* Clear the EXTI line 14 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line14);
		// C14, I_D_INT_A_G_2,
		// Check line C14
#ifndef SPI_SENSOR_TESTING
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
		{
			// Acc data ready nterrupt
			SPI_SensorBufAcc.ui32InterruptDeltaTime = ui32InterruptTime - SPI_SensorBufAcc.ui32InterruptTime;
			SPI_SensorBufAcc.ui32InterruptTime = ui32InterruptTime;
			SPI_SensorBufAcc.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, ACC_GET_DATA);
		}
#endif
	}

	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		/* Clear the EXTI line 15 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line15);
		// C15, I_D_INT_A_G_1
		// Check line C15
#ifndef SPI_SENSOR_TESTING
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
		{
			// Gyro data ready nterrupt
			SPI_SensorBufGyro.ui32InterruptDeltaTime = ui32InterruptTime - SPI_SensorBufGyro.ui32InterruptTime;
			SPI_SensorBufGyro.ui32InterruptTime = ui32InterruptTime;
			SPI_SensorBufGyro.ui16WatchdogCounter = 0;
			RB32_push(&rb32SensorTXQueue, GYRO_GET_DATA);
		}
#endif
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


