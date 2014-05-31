/*
 * sensors_fusion.c
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "parameters.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "math/myMath_pid.h"
#include "sensor_macros.h"
#include "sensor_typedefs.h"
#include "sensors_fusion.h"
#include "gyro.h"
#include "accelerometer.h"
#include "mag.h"
#include "airSpeed.h"
#include "gps.h"
#include "altimeter.h"


ErrorStatus fusion_init(FUSION_CORE *coreData)
{

	if(ERROR == AirSpeed_initDataStructure(&coreData->_airSpeed))
	{
		return ERROR;
	}
	if(ERROR == altimeter_initDataStructure(&coreData->_altimeter))
	{
		return ERROR;
	}
	if(ERROR == gyro_initDataStructure(&coreData->_gyro))
	{
		return ERROR;
	}

	if(ERROR == mag_initDataStructure(&coreData->_mag))
	{
		return ERROR;
	}

	if(ERROR == acc_initDataStructure(&coreData->_accelerometer))
	{
		return ERROR;
	}

	if(ERROR == gps_initData(&coreData->_gps))
	{
		return ERROR;
	}

	// Create identity matrix
	matrix3_init(1, &coreData->_fusion_DCM);

	// Init temperature
	coreData->MPUTemperature = 0;

	// Init system parameters
	coreData->PARAMETERS.systimeToSeconds = SENSOR_SYSTIME_TO_SECONDS;
	coreData->PARAMETERS.minRotation = SENSOR_MIN_ROTATION;
	coreData->PARAMETERS.minRotError = SENSOR_MIN_ROT_ERROR;

	return SUCCESS;
}

ErrorStatus fusion_dataUpdate(void *data, FUSION_SENSORDATA *sensorData)
{
	// Update temperature
	fusion_calculateMPUTemperature((FUSION_CORE *)data, sensorData->data.temperature, sensorData->data.dataTakenTime);
	// Update gyro
	gyro_update((FUSION_CORE *)data, (int16_t*)&sensorData->arrays.gyro, sensorData->data.dataTakenTime);
	// Update rotation matrix
	fusion_updateRotationMatrix((FUSION_CORE *)data);
	// Update acceleration
	acc_update((FUSION_CORE *)data, (int16_t*)&sensorData->arrays.acc, sensorData->data.dataTakenTime);
	// Update speed from acceleration
	acc_updateSpeedCalculation((FUSION_CORE *)data, sensorData->data.dataTakenTime);
	// Update mag
	mag_update((FUSION_CORE *)data, (int16_t*)&sensorData->arrays.mag, sensorData->data.dataTakenTime);
	// Update altimeter
	altimeter_update((FUSION_CORE *)data, sensorData->arrays.pressure.statusPressure, sensorData->arrays.baroTemperatureDegrees, sensorData->arrays.baroTemperatureFrac, sensorData->data.dataTakenTime);
	// Estimate gyro error
	fusion_updateGyroError((FUSION_CORE *)data);


	return SUCCESS;
}

// Function calculates temperature from MPU 6000 temperature measurement.
ErrorStatus fusion_calculateMPUTemperature(FUSION_CORE *data, int16_t temperatureData, uint32_t dataTime)
{
	data->MPUTemperature = ((float32_t) temperatureData / 340)+36.53f;
	return SUCCESS;
}

// Function generates rotation update matrix.
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix)
{
	ErrorStatus status = ERROR;

	// First create identity matrix
	matrix3_init(1, updateMatrix);
	// Populate elements of matrix
	updateMatrix->a.y = -omega->z;
	updateMatrix->a.z = omega->y;

	updateMatrix->b.x = omega->z;
	updateMatrix->b.z = -omega->x;

	updateMatrix->c.x = -omega->y;
	updateMatrix->c.y = omega->x;

	status = SUCCESS;
	return status;
}

// Function uses sensor data to calculate error in DCM estimation
ErrorStatus fusion_updateGyroError(FUSION_CORE *data)
{

	ErrorStatus status;
	status = SUCCESS;
	Vectorf temporaryVector = vectorf_init(0);
	Vectorf temporaryVector1 = vectorf_init(0);
	Vectorf temporaryVector2 = vectorf_init(0);
	Vectorf error_gps_acc = vectorf_init(0);				// Error from GPS and accelerometer data
	Vectorf error_acc_gravity = vectorf_init(0);			// Error from acceleration and gravity
	Vectorf error = vectorf_init(0);						// Cumulative error
	float32_t fTemp = 0;
	float32_t dt = 0;
	float32_t GPSSpeed = 0;

	// Calculate GPS speed
	GPSSpeed = vectorf_getNorm(&data->_gps.speed3D);

	// Check if we have valid GPS and accelerometer data
	// We need speed so do not use if below some value
	if((1 ==data->_gps.valid)&&(1 == data->_accelerometer.valid)&&(param_min_airspeed < GPSSpeed))
	{
		// Use GPS and accelerometer speed data to calculate DCM error in earth frame
		// Calculate 1/dt
		fTemp = 1 / data->_accelerometer.speed_3D_dt;
		// Calculate average acceleration of accelerometer
		status = vectorf_scalarProduct(&data->_accelerometer.Speed_3D, fTemp, &temporaryVector);
		// Calculate [0,0,1] - average acceleration of GPS
		status = vectorf_scalarProduct(&data->_gps.speed3D, -fTemp, &temporaryVector2);
		temporaryVector2.z = temporaryVector2.z + 1;
		// Calculate error
		status = vectorf_crossProduct(&temporaryVector, &temporaryVector2, &error_gps_acc);
		// Make yaw error z = 0 - we will calculate that error from heading
		error_gps_acc.z = 0;

		// GPS data is OK so calculate yaw error from GPS data -> temporaryVector
		// Get plane X vector in earth coordinates - that's where we are pointing
		// Take plane X in earth coordinates - that's column 1 (i think)
		// Set Z to zero - we don't need it
		temporaryVector.z = 0;
		// Normalize vector to length 1
		status = vectorf_normalize(&temporaryVector);

		// Calculate GPS heading vector from GPS angle -> temporaryVector1
		// x = cos(x)
		// y = sin(y)
		// Normalize this vector to length 1
		status = vectorf_normalize(&temporaryVector1);

		// Error is cross product between these two vectors
		status = vectorf_crossProduct(&temporaryVector, &temporaryVector1, &temporaryVector2);
		// We only need the Z component so copy x and y from total error
		temporaryVector2.x = error_gps_acc.x;
		temporaryVector2.y = error_gps_acc.y;

		// Transform to plane frame with DCM transpose
		status = matrix3_transposeVectorMultiply(&data->_fusion_DCM, &temporaryVector2, &error_gps_acc);

		// Check if all was calculated OK
		if(ERROR == status)
		{
			// There was error, so reset error vector
			error_gps_acc = vectorf_init(0);
		}

	}
	// Check if we have valid gyro and accelerometer data and calculate error
	// Do only if speed below limit
	if((param_min_airspeed > GPSSpeed)&&(1 == data->_accelerometer.valid))
	{
		// Store estimated gravity vector
		temporaryVector.x = data->_fusion_DCM.c.x;
		temporaryVector.y = data->_fusion_DCM.c.y;
		temporaryVector.z = data->_fusion_DCM.c.z;
		status = vectorf_crossProduct(&temporaryVector, &data->_accelerometer.vector, &error_acc_gravity);
		// Check if there was calculation error
		// If yes, set error to 0
		if(ERROR == status)
		{
			error_acc_gravity = vectorf_init(0);
		}
	}

	// Check if we have valid magnetometer data and calculate error

	// Check if we have valid GPS data and calculate yaw error


	//error_yaw_gps

	// Calculate delta time
	dt = (float32_t)data->_gyro.deltaTime;
	dt = dt * param_systime_toseconds;

	// If error above limit, update PID
	fTemp = vectorf_getNorm(&error);
	if(param_min_rot_error < fTemp)
	{
		// Calculate delta time in seconds
		status = math_PID3(&error, dt, &data->_gyroErrorPID);
	}

	return SUCCESS;
}

// Function updates rotation matrix from gyro data. At end it renormalizes and reorthogonalizes matrix.
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data)
{
	ErrorStatus status;
	status = SUCCESS;

	Vectorf omega = vectorf_init(0);						// Rotation value
	Matrixf updateMatrix;									// Update matrix
	Matrixf newMatrix;										// Place to store new DCM matrix
	float32_t dt = 0;


	// Calculate delta time
	dt = (float32_t)data->_gyro.deltaTime;
	dt = dt * data->PARAMETERS.systimeToSeconds;


	// Calculate rotation angle
	// Is gyro value * delta time in seconds
	status = vectorf_scalarProduct(&data->_gyro.vector, dt, &omega);

	// If rotation angle above limit, update rotation matrix
	if(data->PARAMETERS.minRotation < vectorf_getNorm(&omega))
	{
		// Generate update matrix
		status = fusion_generateUpdateMatrix(&omega, &updateMatrix);
		// Multiply DCM and update matrix
		status = matrix3_MatrixMultiply(&updateMatrix, &data->_fusion_DCM, &newMatrix);

		// Renormalize and orthogonalize DCM matrix
		status = matrix3_normalizeOrthogonalizeMatrix(&newMatrix, param_dcm_max_orth_error);

		// Check if matrix is OK and copy data
		if(SUCCESS == status)
		{
			matrix3_copy(&newMatrix, &data->_fusion_DCM);
		}
	}
	return status;
}

