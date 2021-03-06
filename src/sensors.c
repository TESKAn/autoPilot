/*
 * sensors.c
 *
 *  Created on: Oct 31, 2012
 *      Author: Jure
 */

#include "allinclude.h"
#include "sensors/altimeter.h"

volatile uint16_t I2C2_ProcesState = 0;
volatile Flag I2C2_Flags;
volatile uint16_t I2C2_StartReg = 0;
volatile uint16_t I2C2_ReadData = 0;
volatile uint16_t I2C2_WriteData = 0;
volatile uint8_t I2C2_DeviceAddress = 0;
uint8_t I2C2_DMABufTX[DMA_BUF_COUNT];
uint8_t I2C2_DMABufRX[DMA_BUF_COUNT];
FUSION_SENSORDATA I2C2_sensorBufRX;
volatile int I2C2_DMABufTXCount = 0;
volatile int I2C2_DMABufRXCount = 0;
volatile int I2C2_PollTimer = 0;
volatile int sensorTimeCounter = 0;
volatile uint32_t sensoruTimeCounter = 0;
// Data taken at time
uint32_t sensorAcquisitionTime = 0;

// Offset registers
uint8_t gyroOffsetSampleCount = 0;
uint8_t accOffsetSampleCount = 0;
uint8_t magOffsetSampleCount = 0;



// Timeout function
void sensorTimer(void)
{
	sensoruTimeCounter++;
	if(sensoruTimeCounter > 100)
	{
		sensoruTimeCounter = 0;
		// Function is called once every millisecond
		sensorTimeCounter++;
		/*
#ifdef DEBUG_USB
		if(sensorTimeCounter > 2)
		{
			int16ToStr(sensorTimeCounter, "Timer=", StringBuffer);
			sendUSBMessage(StringBuffer);
		}
#endif*/
		if(sensorTimeCounter > 65000)
		{
			sensorTimeCounter = 65000;
		}
#ifdef DEBUG_USB
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			sendUSBMessage("I2C timeout");
		}
#endif
	}
	Delaynus(2);
}

void sensorInterruptTimer(void)
{
	// Function is called once every millisecond
	if(TIMEOUT_USEINTERRUPT)
	{
		/*
		sensorTimeCounter++;
		if(sensorTimeCounter > 65000)
		{
			sensorTimeCounter = 65000;
		}*/
	}
}

// Initialize I2C sensors
void sensorInit()
{

	uint8_t retriesCount = 0;
	ErrorStatus error = SUCCESS;
	I2C2_INITDONE = 0;

	// Check if MPU is in sleep mode
	// Read reg 107
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterReceive(MPU6000_ADDRESS, 107, I2C2_DMABufRX, 5);
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
	Delaynus(20000);
	// Check bit 6 - device sleep
	if((I2C2_DMABufRX[0] & _BIT7) == 0)
	{
		// Device was configured, reset MPU
		I2C2_DMABufTX[0] = 107;
		I2C2_DMABufTX[1] = 0x80;
		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
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
		Delaynus(20000);
		I2C2_REENABLE = 1;
	}
	// Enable MPU
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPU6000_Enable(ENABLE);
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
	// Enable I2C bypass to write to HMC5883
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPU6000_EnableI2CBypass(ENABLE);
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
	if(I2C2_REENABLE)
	{
		// Reset slave devices
		// MPL3115
		I2C2_DMABufTX[0] = 38;
		I2C2_DMABufTX[1] = 0x04;
		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPL3115A2_ADDRESS, I2C2_DMABufTX, 2);
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
		Delaynus(20000);
	}
	// Configure HMC5883
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = HMC5883_Enable(ENABLE);
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
	// Configure MPL3115A2
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPL3115A2_Enable(ENABLE);
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
	// Disable I2C bypass
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPU6000_EnableI2CBypass(DISABLE);
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
	// Configure MPU I2C master mode
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPU6000_ConfigureI2CMaster();
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
	// Enable MPU I2C master
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = MPU6000_EnableI2CMaster(ENABLE);
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
	I2C2_INITDONE = 1;
}

// Copies sensor data from I2C to variables
void copySensorData(void)
{

	uint16_t i = 0;
	float32_t fTemp = 0;
	//uint16_t uiTemp = 0;
	// Data is in I2C2_DMABufRX
	// Mark updating sensor data
	SENSORS_UPDATING = 1;

	// Swap bytes to correct endianesse
	byte_swap16(I2C2_sensorBufRX.data.accX);
	byte_swap16(I2C2_sensorBufRX.data.accY);
	byte_swap16(I2C2_sensorBufRX.data.accZ);
	byte_swap16(I2C2_sensorBufRX.data.temperature);
	byte_swap16(I2C2_sensorBufRX.data.gyroX);
	byte_swap16(I2C2_sensorBufRX.data.gyroY);
	byte_swap16(I2C2_sensorBufRX.data.gyroZ);
	byte_swap16(I2C2_sensorBufRX.data.magX);
	byte_swap16(I2C2_sensorBufRX.data.magY);
	byte_swap16(I2C2_sensorBufRX.data.magZ);
	byte_swap32(I2C2_sensorBufRX.data.pressure.statusPressure);

	// Add data time
	I2C2_sensorBufRX.data.dataTakenTime = sensorAcquisitionTime;
	// Call fusion update function
	fusion_dataUpdate(&fusionData, &I2C2_sensorBufRX);


	ACC_X = I2C2_sensorBufRX.data.accX;
	ACC_Y = I2C2_sensorBufRX.data.accY;
	ACC_Z = I2C2_sensorBufRX.data.accZ;

	GYRO_X = I2C2_sensorBufRX.data.gyroX;
	GYRO_Y = I2C2_sensorBufRX.data.gyroY;
	GYRO_Z = I2C2_sensorBufRX.data.gyroZ;

	MAG_X = I2C2_sensorBufRX.data.magX;
	MAG_Y = I2C2_sensorBufRX.data.magY;
	MAG_Z = I2C2_sensorBufRX.data.magZ;

	BARO = (((I2C2_sensorBufRX.data.pressure.statusPressure) & 0x00ffff00) >> 8);


	// Mark end of sensor updating
	SENSORS_UPDATING = 0;

	if(ahrs_data.sampleDiscardCount == 0)
	{
		// Update AHRS
		//ahrs_updateQuaternion();
		ahrs_updateRotationMatrix(&ahrs_data);
	}
	else
	{
		ahrs_data.sampleDiscardCount--;
	}
	// Store angles
	storeAHRSAngles();

	// Check if we are sending constantly
	if(CONSTANT_SERIAL_UPDATE)
	{
		// If yes, send data
		MODBUS_SendMessage();
	}

	// Check if we are saving to log
	if(SD_WRITE_LOG && SCR2_LOGOPEN)
	{
		write_toLog();
	}

	// Update export vars
	//updateExportVars();

	// Check if we are nulling
	if(EXTSENS_NULLING_GYRO)
	{
		nullGyro(&ahrs_data);
	}
	if(EXTSENS_NULLING_ACC)
	{
		nullAcc(&ahrs_data);
	}
	if((USB_OTG_dev.dev.device_status == USB_OTG_CONFIGURED)&&(fastDataSelect != 0))
    {
		// Build two byte data index
		Buffer[0] = 2;
		Buffer[1] = 5;
		Buffer[2] = (fastDataSelect >> 8) & 0x00ff;
		Buffer[3] = fastDataSelect & 0x00ff;
		i = 4;
		if((fastDataSelect & REPORT_DCM) != 0)
		{
			// Store DCM
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_ACC) != 0)&&(i < 61))
		{
			// Store acceleration
			floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_GYRO) != 0)&&(i < 61))
		{
			// Store gyro
			floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_MAG) != 0)&&(i < 61))
		{
			// Store mag
			floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_TC) != 0)&&(i < 61))
		{
			// Store total correction
			floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_RPC) != 0)&&(i < 61))
		{
			// Store RP correction
			floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_YAWC) != 0)&&(i < 61))
		{
			// Store yaw correction
			floatToUint32.f = ahrs_data.YawCorrection.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.YawCorrection.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.YawCorrection.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_GRAVITY) != 0)&&(i < 61))
		{
			// Store gravity
			floatToUint32.f = ahrs_data.GravityVector.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GravityVector.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GravityVector.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_MAG_OFFSET) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.MagOffsetVector.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagOffsetVector.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagOffsetVector.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_PID_I) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.PIData.Ix;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Iy;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Iz;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_PID_P) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.PIData.Px;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Py;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Pz;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_PID_R) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.PIData.Rx;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Ry;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.PIData.Rz;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_XE_B) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_YE_B) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_ZE_B) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_XB_E) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzx];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_YB_E) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzy];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_DCM_ZB_E) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzz];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_GYRO_ADJUSTED) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.GyroValueAdjusted.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GyroValueAdjusted.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.GyroValueAdjusted.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		if(((fastDataSelect & REPORT_MAG_EARTH) != 0)&&(i < 61))
		{
			// Store mag offset
			floatToUint32.f = ahrs_data.MagInEarthFrame.vector.pData[VECT_X];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagInEarthFrame.vector.pData[VECT_Y];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			floatToUint32.f = ahrs_data.MagInEarthFrame.vector.pData[VECT_Z];
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}
		if(((fastDataSelect & REPORT_ADC) != 0)&&(i < 61))
		{
			// Store ADC result in volts
			fTemp = (float32_t) AIN0;
			fTemp = fTemp * 0.0008056640625f;

			floatToUint32.f = fTemp;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			fTemp = (float32_t) AIN1;
			fTemp = fTemp * 0.0008056640625f;
			floatToUint32.f = fTemp;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
			fTemp = (float32_t) AIN2;
			fTemp = fTemp * 0.0008056640625f;
			floatToUint32.f = fTemp;
			Buffer[i] = floatToUint32.ch[0];
			i++;
			Buffer[i] = floatToUint32.ch[1];
			i++;
			Buffer[i] = floatToUint32.ch[2];
			i++;
			Buffer[i] = floatToUint32.ch[3];
			i++;
		}

		while(i < 64)
		{
			Buffer[i] = 0;
			i++;
		}
		USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64);
    }
}

ErrorStatus MPU6000_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	uint8_t retriesCount = 0;
	masterReceive(MPU6000_ADDRESS, 107, I2C2_DMABufRX, 5);
	ui8RegState = I2C2_DMABufRX[0];

	I2C2_DMABufTX[0] = 107;
	if(newState == ENABLE)
	{
		// Set power = ON with clock source = X axis gyro
		ui8RegState = ui8RegState & ~_BIT6;
		I2C2_DMABufTX[1] = ui8RegState | 0x01;
	}
	else
	{
		// Set device to sleep
		I2C2_DMABufTX[1] = ui8RegState | 0x40;
	}
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
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
	if(error == SUCCESS)
	{
		Delaynus(2000);
		// Configure sensors
		// Register 26
		I2C2_DMABufTX[0] = 26;
		// Reg 26 = 0000 0001	Set low pass filter to 184/188 Hz
		I2C2_DMABufTX[1] = 0x01;
		// Reg 27 = 0001 0000	Set gyro maximum rate at 1000 �/sec
		I2C2_DMABufTX[2] = 0x10;
		// Reg 28 = 0001 0000	Set accel maximum rate at 8g
		I2C2_DMABufTX[3] = 0x10;
		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 4);
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
		Delaynus(20000);
		return  error;
	}


	if(error == SUCCESS)
	{
		Delaynus(2000);
		// Configure sensor interrupt
		// Register 55
		I2C2_DMABufTX[0] = 55;
		// Reg 55 = 0001 0000	Interrupt active high, push-pull, 50 us pulse, clear on any read, no fsync, no i2c bypass
		I2C2_DMABufTX[1] = 0x10;
		// Reg 56 = 000 0001	Data ready interrupt enable
		I2C2_DMABufTX[2] = 0x01;

		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 3);
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
		Delaynus(20000);
		return  error;
	}



	else
	{
		return ERROR;
	}
}

ErrorStatus MPU6000_EnableI2CBypass(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	uint8_t retriesCount = 0;
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterReceive(MPU6000_ADDRESS, 55, I2C2_DMABufRX, 5);
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
	if(error == SUCCESS)
	{
		ui8RegState = I2C2_DMABufRX[0];

		I2C2_DMABufTX[0] = 55;
		if(newState == ENABLE)
		{
	    	I2C2_DMABufTX[1] = ui8RegState | 0x02;
		}
		else
		{
			I2C2_DMABufTX[1] = ui8RegState & ~0x02;
		}

		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
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
		Delaynus(20000);
		return  error;
	}
	else
	{
		return ERROR;
	}

}

ErrorStatus MPU6000_ConfigureI2CMaster(void)
{
	ErrorStatus error = ERROR;
	uint8_t retriesCount = 0;
	// Start with reg 36
	I2C2_DMABufTX[0] = 36;
	// I2C master control = 0101 1101; last four bits = I2C clock = 400 kHz
	I2C2_DMABufTX[1] = 0x5d;
	// Slave 0 is magnetometer
	// Registers 37 - 39 - master 0 control
	// 37 - I2C_SLV0_ADDR = 1001 1110
	I2C2_DMABufTX[2] = 0x9E;
	// 38 - I2C_SLV0_REG - start read at this reg
	I2C2_DMABufTX[3] = 0x03;
	// 39 - I2C_SLV0_CTRL = 1000 0110 -> enable, read 6 bytes
	I2C2_DMABufTX[4] = 0x86;
	// Slave 1 is barometer
	// Registers 40 - 42 - master 1 control
	// 40 - I2C SLV1_ADDR = 1110 0000
	I2C2_DMABufTX[5] = 0xE0;
	// 41 = I2C_SLV1_REG - start read at this reg
	I2C2_DMABufTX[6] = 0x00;
	// 42 - I2C_SLV1_CTRL = 1000 0110 -> enable, read 6 bytes, status + pressure + temperature
	I2C2_DMABufTX[7] = 0x86;
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 8);
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
	Delaynus(20000);
	return  error;
}

ErrorStatus MPU6000_EnableI2CMaster(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t ui8RegState = 0;
	uint8_t retriesCount = 0;
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterReceive(MPU6000_ADDRESS, 106, I2C2_DMABufRX, 5);
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
	if(error == SUCCESS)
	{
		ui8RegState = I2C2_DMABufRX[0];

		I2C2_DMABufTX[0] = 106;
		if(newState == ENABLE)
		{
			I2C2_DMABufTX[1] = ui8RegState | 0x20;
		}
		else
		{
			I2C2_DMABufTX[1] = ui8RegState & ~0x20;
		}
		for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
		{
			error = masterSend(MPU6000_ADDRESS, I2C2_DMABufTX, 2);
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
		Delaynus(20000);
		return  error;
	}
	else
	{
		return ERROR;
	}
}

ErrorStatus HMC5883_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t retriesCount = 0;
	// Start at reg 0
	I2C2_DMABufTX[0] = 0;
	// CRA = 8 samples, 75 Hz, normal
	I2C2_DMABufTX[1] = 0x78;
	// CRB = +- 1,3 Gauss
	I2C2_DMABufTX[2] = 0x20;
	if(newState == ENABLE)
	{
		// Mode = continuous
		I2C2_DMABufTX[3] = 0x00;
	}
	else
	{
		// Mode = idle
		I2C2_DMABufTX[3] = 0x02;
	}
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterSend(HMC5883_ADDRESS, I2C2_DMABufTX, 4);
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
	Delaynus(20000);
	return error;
}

ErrorStatus MPL3115A2_Enable(FunctionalState newState)
{
	ErrorStatus error = ERROR;
	uint8_t retriesCount = 0;
	// Start at reg 38
	I2C2_DMABufTX[0] = 38;
	if(newState == ENABLE)
	{
		// Mode = enable, altimeter mode
		//I2C2_DMABufTX[1] = 0x81;
		// Mode = enable, barometer mode
		//I2C2_DMABufTX[1] = 0x01;
		// Barometer, oversampling enabled.
		I2C2_DMABufTX[1] = 0x39;
	}
	else
	{
		// Mode = disable
		//I2C2_DMABufTX[1] = 0x80;
		I2C2_DMABufTX[1] = 0x00;
	}
	//
	I2C2_DMABufTX[2] = 0x00;
	// Interrupt active high, open drain
	I2C2_DMABufTX[3] = 0x33;
	// Data ready interrupt enabled
	I2C2_DMABufTX[4] = 0x80;
	// Interrupt routed to pin 2
	I2C2_DMABufTX[5] = 0x00;
	for(retriesCount = I2C2_ERROR_RETRIESCOUNT; retriesCount > 0; retriesCount --)
	{
		error = masterSend(MPL3115A2_ADDRESS, I2C2_DMABufTX, 6);
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
	Delaynus(20000);
	return error;
}

ErrorStatus masterSend(uint8_t device, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;
	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);
	// Disable DMA TX Channel
	DMA_Cmd(DMA_I2C2_TX, DISABLE);
	// Wait until stream is disabled
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_TX) != DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if((I2C_DMACheckForError(DMA_I2C2_TX) == ERROR))//||(sensorTimeCounter > I2C2_ERRORTIMEOUT))
		{
			return ERROR;
			break;
		}
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_TX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_TX, &DMAInitStructure);

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_TX, DMA_FLAG_TCIF7 | DMA_FLAG_FEIF7 | DMA_FLAG_DMEIF7 |  DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		sensorTimeCounter = 0;
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{
			// Call sensor timer
			sensorTimer();
			if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
			{
				return ERROR;
				break;
			}
			// Check for I2C error
			if(I2C_CheckForError(I2C2) == ERROR)
			{
				return ERROR;
				break;
			}
		}
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}


	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	// Transfer DMA data

	DMA_ClearFlag(DMA_I2C2_TX, DMA_FLAG_TCIF7);
	I2C2_DMA_ClearErrors();

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA TX Channel */
	DMA_Cmd(DMA_I2C2_TX, ENABLE);

	/* Wait until I2Cx_DMA_STREAM_RX enabled or time out */
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_TX)!= ENABLE)
	{
		// Check if we have complete interrupt
		if(DMA_GetFlagStatus(DMA_I2C2_TX,DMA_FLAG_TCIF7)!=RESET)
		{
			// If yes, break
			break;
		}
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_TX) == ERROR)
		{
			return ERROR;
			break;
		}
	}


	/* Transfer complete or time out */
	sensorTimeCounter = 0;
	while (DMA_GetFlagStatus(DMA_I2C2_TX,DMA_FLAG_TCIF7)==RESET)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_TX) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	// Check TxE bit
	sensorTimeCounter = 0;
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXE) == RESET)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA TX Channel */
	DMA_Cmd(DMA_I2C2_TX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_TX disabled or time out */
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_TX)!= DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_TX) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	return SUCCESS;
}

// Function starts DMA receive process
ErrorStatus masterReceive_beginDMA(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;

	// Mark receive in progress
	I2C2_WAITINGDATA = 1;
	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Wait until stream is disabled
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_RX) != DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_RX) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_RX, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		sensorTimeCounter = 0;
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{
			// Call sensor timer
			sensorTimer();
			if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
			{
				return ERROR;
				break;
			}
			// Check for I2C error
			if(I2C_CheckForError(I2C2) == ERROR)
			{
				return ERROR;
				break;
			}
		}
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	I2C_SendData(I2C2, startReg);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}


	/* Send I2Cx START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2Cx EV5 and clear it or time out*/
	sensorTimeCounter = 0;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* Send I2Cx slave Address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on I2Cx EV6 and clear it or time out */
	sensorTimeCounter = 0;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, ENABLE);

	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);
	return SUCCESS;
}

ErrorStatus masterReceive(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount)
{
	DMA_InitTypeDef DMAInitStructure;

	// Disable I2C2 interrupts
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Wait until stream is disabled
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_RX) != DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_RX) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Configure I2C2 DMA
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;//    I2C2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)dataBuffer;
	DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMAInitStructure.DMA_BufferSize = byteCount;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// Configure peripheral
	DMA_Init(DMA_I2C2_RX, &DMAInitStructure);

	/* Master Receiver -----------------------------------------------------------*/

	// Clear DMA flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 |  DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3);

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);

	// Check BUSY flag
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		sensorTimeCounter = 0;
		while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
		{
			// Call sensor timer
			sensorTimer();
			if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
			{
				return ERROR;
				break;
			}
			// Check for I2C error
			if(I2C_CheckForError(I2C2) == ERROR)
			{
				return ERROR;
				break;
			}
		}
	}

	// Send I2C1 START condition
	I2C_GenerateSTART(I2C2, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	// Send slave Address for write
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Transmitter);
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	I2C_SendData(I2C2, startReg);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	sensorTimeCounter = 0;
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	/* Send I2Cx START condition */
	I2C_GenerateSTART(I2C2, ENABLE);

	/* Test on I2Cx EV5 and clear it or time out*/
	sensorTimeCounter = 0;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	/* Send I2Cx slave Address for read */
	I2C_Send7bitAddress(I2C2, device, I2C_Direction_Receiver);

	/* Test on I2Cx EV6 and clear it or time out */
	sensorTimeCounter = 0;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for I2C error
		if(I2C_CheckForError(I2C2) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	/* I2Cx DMA Enable */
	I2C_DMACmd(I2C2, ENABLE);

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, ENABLE);

	/* Wait until I2Cx_DMA_STREAM_RX enabled or time out */
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= ENABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_RX) == ERROR)
		{
			return ERROR;
			break;
		}
	}

	/* Transfer complete or time out */
	sensorTimeCounter = 0;
	while (DMA_GetFlagStatus(DMA_I2C2_RX,DMA_FLAG_TCIF3)==RESET)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_RX) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* Send I2Cx STOP Condition */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_I2C2_RX, DISABLE);

	/* Wait until I2Cx_DMA_STREAM_RX disabled or time out */
	sensorTimeCounter = 0;
	while (DMA_GetCmdStatus(DMA_I2C2_RX)!= DISABLE)
	{
		// Call sensor timer
		sensorTimer();
		if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
		{
			return ERROR;
			break;
		}
		// Check for DMA error
		if(I2C_DMACheckForError(DMA_I2C2_RX) == ERROR)
		{
			return ERROR;
			break;
		}
	}
	/* Disable I2C DMA request */
	I2C_DMACmd(I2C2,DISABLE);
	return SUCCESS;
}

// Function checks for errors in I2C DMA peripheral
ErrorStatus I2C_DMACheckForError(DMA_Stream_TypeDef* DMAy_Streamx)
{
	ErrorStatus error = SUCCESS;
	if(DMAy_Streamx == DMA_I2C2_TX)	// Stream 7
	{
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_I2C2_TX_TEIF))
		{
			I2C2_DMA_TX_TXERR = 1;
			error = ERROR;
		}
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_I2C2_TX_DMEIF))
		{
			I2C2_DMA_TX_DMEIF = 1;
			error = ERROR;
		}
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_I2C2_TX_FEIF))
		{
			//I2C2_DMA_TX_FEIF = 1;
			//error = ERROR;
		}
		if(sensorTimeCounter > I2C2_DMA_TIMEOUT_TIME)
		{
			I2C2_DMA_TIMEOUT = 1;
			error = ERROR;
		}
	}
	else if(DMAy_Streamx == DMA_I2C2_RX)	// Stream 3
	{
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_FLAG_TEIF3))
		{
			I2C2_DMA_RX_TXERR = 1;
			error = ERROR;
		}
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_FLAG_DMEIF3))
		{
			I2C2_DMA_RX_DMEIF = 1;
			error = ERROR;
		}
		if(DMA_GetFlagStatus(DMAy_Streamx, DMA_FLAG_FEIF3))
		{
			//I2C2_DMA_RX_FEIF = 1;
			//error = ERROR;
		}
		if(sensorTimeCounter > I2C2_DMA_TIMEOUT_TIME)
		{
			I2C2_DMA_TIMEOUT = 1;
			error = ERROR;
		}
	}
	return error;
}

// Function checks for errors in I2C peripheral
ErrorStatus I2C_CheckForError(I2C_TypeDef* I2Cx)
{

	ErrorStatus error = SUCCESS;
	// Check timeout flag
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_TIMEOUT = 1;
		}
		error = ERROR;
	}
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_PECERR))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_PEC = 1;
		}
		error = ERROR;
	}
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_OVR))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_OVR = 1;
		}
		error = ERROR;
	}
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_AF = 1;
		}
		error = ERROR;
	}
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ARLO))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_ARLO = 1;
		}
		error = ERROR;
	}
	if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BERR))
	{
		if(I2Cx == I2C2)
		{
			I2C2_ERROR_BERR = 1;
		}
		error = ERROR;
	}
	if(sensorTimeCounter > I2C2_ERRORTIMEOUT)
	{
		if(I2Cx == I2C2)
		{
			I2C2_TIMEOUT = 1;
		}
		error = ERROR;
	}
	return error;
}

void I2C2_Configure(FunctionalState NewState)
{
	I2C_InitTypeDef I2CInitStruct;
	//make structure for configuring pins
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Config GPIO
	// Connect pins B10 and B11 to I2C 2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
	// Select pins 10 and 11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	// I2C2 config
	// Configure I2C 2 for sensor communication
	// Set clock to 400 kHz
	I2CInitStruct.I2C_ClockSpeed = 400000;
	I2CInitStruct.I2C_Mode = I2C_Mode_I2C;
	I2CInitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2CInitStruct.I2C_OwnAddress1 = 0x00;
	I2CInitStruct.I2C_Ack = I2C_Ack_Enable;
	I2CInitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2CInitStruct);
	// Configure interrupts
	// Enable event interrupt and buf empty interrupt event and error interrupt event
	// Do not enable BUF interrupt if using DMA
	// I2C_IT_BUF, I2C_IT_EVT, I2C_IT_ERR
	//I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, ENABLE);
	// Configure DMA
	DMA_DeInit(DMA_I2C2_TX);
	DMA_DeInit(DMA_I2C2_RX);
	// Configure DMA1 stream 3 transfer complete interrupt
	//DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);
	// Configure DMA1 stream 7 transfer complete interrupt
	//DMA_ITConfig(DMA1_Stream7, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE, ENABLE);
	// Enable I2C 2
	I2C_Cmd(I2C2, NewState);
}


void I2C2_ResetInterface(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	// Set pins to input
	// Connect pins B10 and B11 to I2C 2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
	// Select pins 10 and 11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	// Disable I2C2 interface
	I2C_Cmd(I2C2, DISABLE);
	// Clear all error flags
	I2C_ClearFlag(I2C2, I2C_FLAG_SMBALERT);
	I2C_ClearFlag(I2C2, I2C_FLAG_TIMEOUT);
	I2C_ClearFlag(I2C2, I2C_FLAG_PECERR);
	I2C_ClearFlag(I2C2, I2C_FLAG_OVR);
	I2C_ClearFlag(I2C2, I2C_FLAG_AF);
	I2C_ClearFlag(I2C2, I2C_FLAG_ARLO);
	I2C_ClearFlag(I2C2, I2C_FLAG_BERR);
	// Clear all interrupts
	I2C_ClearITPendingBit(I2C2, I2C_IT_SMBALERT);
	I2C_ClearITPendingBit(I2C2, I2C_IT_TIMEOUT);
	I2C_ClearITPendingBit(I2C2, I2C_IT_PECERR);
	I2C_ClearITPendingBit(I2C2, I2C_IT_OVR);
	I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
	I2C_ClearITPendingBit(I2C2, I2C_IT_ARLO);
	I2C_ClearITPendingBit(I2C2, I2C_IT_BERR);
	// Deinit interface
	I2C_DeInit(I2C2);
	// Disable DMA interface
	// Disable DMA RX Channel
	DMA_Cmd(DMA_I2C2_RX, DISABLE);
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_RX);
	// Clear DMA RX flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_TCIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_HTIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_TEIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_DMEIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_FEIF);
	// Clear DMA interrupts
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_TCIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_HTIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_TEIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_DMEIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_FEIF);

	// Disable DMA TX Channel
	DMA_Cmd(DMA_I2C2_TX, DISABLE);
	// Deinit DMA
	DMA_DeInit(DMA_I2C2_TX);
	// Clear DMA TX flags
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_TCIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_HTIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_TEIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_DMEIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_FEIF);
	// Clear DMA interrupts
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_TCIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_HTIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_TEIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_DMEIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_FEIF);

	// Set pins to I2C2
	// Connect pins B10 and B11 to I2C 2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
	// Select pins 10 and 11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	//set output type
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	// open drain output
	//set pull-up
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//set pin mode to alternate function
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//set pin speed
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//write mode to selected pins and selected port
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Reconfigure and enable I2C2 interface
	I2C2_Configure(ENABLE);

}

void I2C2_DMA_ClearErrors(void)
{
	// Clear DMA RX flags
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_TCIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_HTIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_TEIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_DMEIF);
	DMA_ClearFlag(DMA_I2C2_RX, DMA_I2C2_RX_FEIF);
	// Clear DMA interrupts
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_TCIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_HTIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_TEIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_DMEIF);
	DMA_ClearITPendingBit(DMA_I2C2_RX, DMA_I2C2_RX_IT_FEIF);
	// Clear DMA TX flags
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_TCIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_HTIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_TEIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_DMEIF);
	DMA_ClearFlag(DMA_I2C2_TX, DMA_I2C2_TX_FEIF);
	// Clear DMA interrupts
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_TCIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_HTIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_TEIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_DMEIF);
	DMA_ClearITPendingBit(DMA_I2C2_TX, DMA_I2C2_TX_IT_FEIF);
}

void nullGyro(AHRSData * data)
{
	if(EXTSENS_NULLING_GYRO)
	{
		// Check if samples are OK
		if(((int16_t)GYRO_X < OFFSET_SAMPLE_MAX_VALUE)&&
				((int16_t)GYRO_Y < OFFSET_SAMPLE_MAX_VALUE)&&
				((int16_t)GYRO_Z < OFFSET_SAMPLE_MAX_VALUE)&&
				((int16_t)GYRO_X > OFFSET_SAMPLE_MIN_VALUE)&&
				((int16_t)GYRO_Y > OFFSET_SAMPLE_MIN_VALUE)&&
				((int16_t)GYRO_Z > OFFSET_SAMPLE_MIN_VALUE))
		{
			// Store current values as offsets
			data->GyroOffsetVector.vector.pData[VECT_X] += ((int16_t)GYRO_X >> 1);
			data->GyroOffsetVector.vector.pData[VECT_Y] += ((int16_t)GYRO_Y >> 1);
			data->GyroOffsetVector.vector.pData[VECT_Z] += ((int16_t)GYRO_Z >> 1);
			gyroOffsetSampleCount++;
		}
		else
		{
#ifdef DEBUG_USB
				sendUSBMessage("Bad value, nulling restart");
#endif
			// Else restart process
			gyroOffsetSampleCount = 0;
			data->GyroOffsetVector.vector.pData[VECT_X] = 0;
			data->GyroOffsetVector.vector.pData[VECT_Y] = 0;
			data->GyroOffsetVector.vector.pData[VECT_Z] = 0;
		}

		if(gyroOffsetSampleCount >= OFFSET_SAMPLE_COUNT)
		{
			// Check that values are below OFFSET_MAX_VALUE
			if((data->GyroOffsetVector.vector.pData[VECT_X] < OFFSET_MAX_VALUE)&&
					(data->GyroOffsetVector.vector.pData[VECT_Y] < OFFSET_MAX_VALUE)&&
					(data->GyroOffsetVector.vector.pData[VECT_Z] < OFFSET_MAX_VALUE)&&
					(data->GyroOffsetVector.vector.pData[VECT_X] > OFFSET_MIN_VALUE)&&
					(data->GyroOffsetVector.vector.pData[VECT_Y] > OFFSET_MIN_VALUE)&&
					(data->GyroOffsetVector.vector.pData[VECT_Z] > OFFSET_MIN_VALUE))
			{
				EXTSENS_NULLING_GYRO = 0;
#ifdef DEBUG_USB
				sendUSBMessage("Nulling done");
#endif
			}
			// Else redo nulling
			else
			{
				gyroOffsetSampleCount = 0;
				data->GyroOffsetVector.vector.pData[VECT_X] = 0;
				data->GyroOffsetVector.vector.pData[VECT_Y] = 0;
				data->GyroOffsetVector.vector.pData[VECT_Z] = 0;
			}
		}
	}
}

void nullAcc(AHRSData * data)
{
	// Do not null Z value -> gravity
	if(EXTSENS_NULLING_ACC)
	{
		// Check if samples are OK
		if(((int16_t)ACC_X < OFFSET_SAMPLE_MAX_VALUE)&&
				((int16_t)ACC_Y < OFFSET_SAMPLE_MAX_VALUE)&&
				((int16_t)ACC_X > OFFSET_SAMPLE_MIN_VALUE)&&
				((int16_t)ACC_Y > OFFSET_SAMPLE_MIN_VALUE))
		{
			// Store current values as offsets
			data->AccOffsetVector.vector.pData[VECT_X] += ((int16_t)ACC_X >> 1);
			data->AccOffsetVector.vector.pData[VECT_Y] += ((int16_t)ACC_Y >> 1);
			accOffsetSampleCount++;
		}
		else
		{
#ifdef DEBUG_USB
				sendUSBMessage("Bad value, nulling restart");
#endif
			// Else restart process
			accOffsetSampleCount = 0;
			data->AccOffsetVector.vector.pData[VECT_X] = 0;
			data->AccOffsetVector.vector.pData[VECT_Y] = 0;
			data->AccOffsetVector.vector.pData[VECT_Z] = 0;
		}

		if(accOffsetSampleCount >= OFFSET_SAMPLE_COUNT)
		{
			// Check that values are below OFFSET_MAX_VALUE
			if((data->AccOffsetVector.vector.pData[VECT_X] < OFFSET_MAX_VALUE)&&
					(data->AccOffsetVector.vector.pData[VECT_Y] < OFFSET_MAX_VALUE)&&
					(data->AccOffsetVector.vector.pData[VECT_X] > OFFSET_MIN_VALUE)&&
					(data->AccOffsetVector.vector.pData[VECT_Y] > OFFSET_MIN_VALUE))
			{
				EXTSENS_NULLING_ACC = 0;
#ifdef DEBUG_USB
				sendUSBMessage("Nulling done");
#endif
			}
			// Else redo nulling
			else
			{
				accOffsetSampleCount = 0;
				data->AccOffsetVector.vector.pData[VECT_X] = 0;
				data->AccOffsetVector.vector.pData[VECT_Y] = 0;
				data->AccOffsetVector.vector.pData[VECT_Z] = 0;
			}
		}
	}
}


