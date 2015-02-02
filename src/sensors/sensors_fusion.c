/*
 * sensors_fusion.c
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
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
#include "functions.h"

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

	// Init maximum gyro amplitude when updating error
	coreData->maxGyroErrorAmplitude = GYRO_MAX_ERROR_AMPLITUDE;
	coreData->gyroFastRotation = GYRO_FAST_ROTATION;
	coreData->maxGyroErrorUpdateRate = GYRO_MAX_ERROR_UPDATE_RATE;

	// Init PIDs
	// Gyro drift
	math_PIDInit(&coreData->_gyroErrorPID.x, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD);
	math_PIDInit(&coreData->_gyroErrorPID.y, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD);
	math_PIDInit(&coreData->_gyroErrorPID.z, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD);

	coreData->_gyroErrorPID.x.errIMax = GYRO_PID_DRIFT_IMAX;
	coreData->_gyroErrorPID.x.errIMin = GYRO_PID_DRIFT_IMIN;
	coreData->_gyroErrorPID.x.outMax = GYRO_PID_DRIFT_SMAX;
	coreData->_gyroErrorPID.x.outMin = GYRO_PID_DRIFT_SMIN;
	coreData->_gyroErrorPID.x.errMax = GYRO_PID_DRIFT_EMAX;
	coreData->_gyroErrorPID.x.errMin = GYRO_PID_DRIFT_EMIN;

	coreData->_gyroErrorPID.y.errIMax = GYRO_PID_DRIFT_IMAX;
	coreData->_gyroErrorPID.y.errIMin = GYRO_PID_DRIFT_IMIN;
	coreData->_gyroErrorPID.y.outMax = GYRO_PID_DRIFT_SMAX;
	coreData->_gyroErrorPID.y.outMin = GYRO_PID_DRIFT_SMIN;
	coreData->_gyroErrorPID.y.errMax = GYRO_PID_DRIFT_EMAX;
	coreData->_gyroErrorPID.y.errMin = GYRO_PID_DRIFT_EMIN;

	coreData->_gyroErrorPID.z.errIMax = GYRO_PID_DRIFT_IMAX;
	coreData->_gyroErrorPID.z.errIMin = GYRO_PID_DRIFT_IMIN;
	coreData->_gyroErrorPID.z.outMax = GYRO_PID_DRIFT_SMAX;
	coreData->_gyroErrorPID.z.outMin = GYRO_PID_DRIFT_SMIN;
	coreData->_gyroErrorPID.z.errMax = GYRO_PID_DRIFT_EMAX;
	coreData->_gyroErrorPID.z.errMin = GYRO_PID_DRIFT_EMIN;



	// Gyro gain
	math_PIDInit(&coreData->_gyroGainPID.x, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD);
	math_PIDInit(&coreData->_gyroGainPID.y, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD);
	math_PIDInit(&coreData->_gyroGainPID.z, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD);

	coreData->_gyroGainPID.x.errIMax = GYRO_PID_GAIN_IMAX;
	coreData->_gyroGainPID.x.errIMin = GYRO_PID_GAIN_IMIN;
	coreData->_gyroGainPID.x.outMax = GYRO_PID_GAIN_SMAX;
	coreData->_gyroGainPID.x.outMin = GYRO_PID_GAIN_SMIN;
	coreData->_gyroGainPID.x.errMax = GYRO_PID_GAIN_EMAX;
	coreData->_gyroGainPID.x.errMin = GYRO_PID_GAIN_EMIN;

	coreData->_gyroGainPID.y.errIMax = GYRO_PID_GAIN_IMAX;
	coreData->_gyroGainPID.y.errIMin = GYRO_PID_GAIN_IMIN;
	coreData->_gyroGainPID.y.outMax = GYRO_PID_GAIN_SMAX;
	coreData->_gyroGainPID.y.outMin = GYRO_PID_GAIN_SMIN;
	coreData->_gyroGainPID.y.errMax = GYRO_PID_GAIN_EMAX;
	coreData->_gyroGainPID.y.errMin = GYRO_PID_GAIN_EMIN;

	coreData->_gyroGainPID.z.errIMax = GYRO_PID_GAIN_IMAX;
	coreData->_gyroGainPID.z.errIMin = GYRO_PID_GAIN_IMIN;
	coreData->_gyroGainPID.z.outMax = GYRO_PID_GAIN_SMAX;
	coreData->_gyroGainPID.z.outMin = GYRO_PID_GAIN_SMIN;
	coreData->_gyroGainPID.z.errMax = GYRO_PID_GAIN_EMAX;
	coreData->_gyroGainPID.z.errMin = GYRO_PID_GAIN_EMIN;

	// Create identity matrix
	matrix3_init(1, &coreData->_fusion_DCM);

	matrix3_init(1, &coreData->_GPS_DCM);

	// Init temperature
	coreData->MPUTemperature = 0;

	// Init system parameters
	coreData->PARAMETERS.systimeToSeconds = SENSOR_SYSTIME_TO_SECONDS;
	coreData->PARAMETERS.minRotation = SENSOR_MIN_ROTATION;
	coreData->PARAMETERS.minRotError = SENSOR_MIN_ROT_ERROR;
	coreData->PARAMETERS.minGPSSpeed = SENSOR_MIN_GPS_SPEED;
	coreData->PARAMETERS.gyroErrorUpdateInterval = GYRO_ERROR_UPDATE_INTERVAL;
	coreData->PARAMETERS.gyroIErrorUpdateInterval = GYRO_I_UPDATE_INTERVAL;


	// Set PID for first run
	fusion_initGyroDriftPID(&fusionData);
	fusion_initGyroGainPID(&fusionData);

	return SUCCESS;
}

ErrorStatus fusion_initGyroDriftPID(FUSION_CORE *data)
{
	data->_gyroErrorPID.x.im = 0.023f / data->_gyroErrorPID.x.Ki;
	data->_gyroErrorPID.y.im = 0.01f / data->_gyroErrorPID.y.Ki;
	data->_gyroErrorPID.z.im = -0.023f / data->_gyroErrorPID.z.Ki;

	// And default output
	data->_gyroErrorPID.x.s = 0.023;
	data->_gyroErrorPID.y.s = 0.01;
	data->_gyroErrorPID.z.s = -0.023;
	return SUCCESS;
}

ErrorStatus fusion_initGyroGainPID(FUSION_CORE *data)
{
	// Init default I value
	data->_gyroGainPID.x.im = 0.030517578125f / data->_gyroGainPID.x.Ki;
	data->_gyroGainPID.y.im = 0.030517578125f / data->_gyroGainPID.y.Ki;
	data->_gyroGainPID.z.im = 0.030517578125f / data->_gyroGainPID.z.Ki;

	// And default output
	data->_gyroGainPID.x.s = 0.030517578125f;
	data->_gyroGainPID.y.s = 0.030517578125f;
	data->_gyroGainPID.z.s = 0.030517578125f;

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
		Vectorf error_mag = vectorf_init(0);					// Error from compass
		Vectorf error = vectorf_init(0);						// Cumulative error
		float32_t fTemp = 0;
		float32_t dt = 0;
		float32_t GPSSpeed = 0;

		// Calculate GPS speed
		GPSSpeed = vectorf_getNorm(&data->_gps.speed3D);

		// Check if we have valid GPS and accelerometer data
		// We need speed so do not use if below some value
		/*
		if((1 ==data->_gps.valid)&&(1 == data->_accelerometer.valid)&&(data->PARAMETERS.minGPSSpeed < GPSSpeed))
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

		}*/
		// Check if we have valid gyro and accelerometer data and calculate error
		// Do only if speed below limit and acceleration is really low
		if((data->PARAMETERS.minGPSSpeed > GPSSpeed)&&(1 == data->_accelerometer.valid))
		{
			// Check that we have only gravity - acc. magnitude is 1 +- 0.05
			fTemp = vectorf_getNorm(&data->_accelerometer.vector);
			if((0.95 < fTemp)&&(1.05 > fTemp))
			{
				// Check that gyro amplitude is less than set limit for updating gyro error
				//fTemp = vectorf_getNorm(&data->_gyro.vector);
				//if(fTemp < data->maxGyroErrorAmplitude)
				{
					// Store estimated gravity vector
					/*
					temporaryVector.x = data->_fusion_DCM.c.x;
					temporaryVector.y = data->_fusion_DCM.c.y;
					temporaryVector.z = data->_fusion_DCM.c.z;
					*/
					status = vectorf_crossProduct(&data->_fusion_DCM.c, &data->_accelerometer.vectorNormalized, &error_acc_gravity);
					// Check if there was calculation error
					// If yes, set error to 0
					if(ERROR == status)
					{
						error_acc_gravity = vectorf_init(0);
					}
				}
			}
			else
			{
				error_acc_gravity = vectorf_init(0);
			}
		}
		// Yaw error from mag heading
		/*
		if (data->_mag.valid)
		{
			// Use euler yaw angle - fusionData.ROLLPITCHYAW.yaw
			fTemp = data->_mag.heading - data->ROLLPITCHYAW.yaw;
			// fTemp is yaw error z in earth frame
			// Convert to body frame
			error_mag.x = 0;
			error_mag.y = 0;
			error_mag.z = fTemp * -0.1f;

			// Errors calculated in earth frame, transform to body frame
			matrix3_transposeVectorMultiply(&data->_fusion_DCM, &error_mag, &error_mag);



		}
*/

		// Check if we have valid magnetometer data and calculate yaw error
		if(data->_mag.valid)
		{

			// Calculate heading error in body frame
			// Use gravity from accelerometer
			// grav X raw mag = earth Y axis
			// earth Y axis X DCM line b is yaw error in body frame
			// temporaryVector - normalized mag with subtracted offsets
			status = vectorf_normalizeAToB(&data->_mag.vector, &temporaryVector);
			// Get where earth's Y axis is supposed to be


			status = vectorf_crossProduct(&temporaryVector, &data->_accelerometer.vectorNormalized, &data->_mag.earthYAxis);
			// Normalize
			vectorf_normalize(&data->_mag.earthYAxis);
			// Get error - rotate DCM B axis to our calculated Y axis
			status = vectorf_crossProduct(&data->_mag.earthYAxis, &data->_fusion_DCM.b, &error_mag);
			// Scale error
			//vectorf_scalarProduct(&error_mag, 0.5f, &error_mag);

/*
			// Check roll/pitch error
			fTemp = vectorf_getNorm(&error_acc_gravity);
			if(fTemp < 0.1)
			{
				// Calculate heading error from mag
				// Yaw error is mag in earth frame, normalized, take -y as yaw error
				// Scale
				// Because cross product of [x,y,0] with [1,0,0] is [0,0,-y]
				error_mag.x = 0;
				error_mag.y = 0;
				error_mag.z = data->_mag.vectorEarthFrame.y * -0.1f;

				// Errors calculated in earth frame, transform to body frame
				matrix3_transposeVectorMultiply(&data->_fusion_DCM, &error_mag, &error_mag);
			}*/
		}


		data->_mag.vectorEarthFrame.x = error_mag.x;
		data->_mag.vectorEarthFrame.y = error_mag.y;
		data->_mag.vectorEarthFrame.z = error_mag.z;
		// Check if we have valid GPS data and calculate yaw error


		// Sum up all errors
		status = vectorf_add(&error_acc_gravity, &error_gps_acc, &error);

		status = vectorf_add(&error_mag, &error, &error);

		// Scale error
		vectorf_scalarProduct(&error, data->_gyro.errorScale, &error);

		// Store errors
		vectorf_copy(&error, &data->_gyro.gyroError);

		// Calculate delta time
		dt = (float32_t)data->_gyro.deltaTime;
		dt = dt * data->PARAMETERS.systimeToSeconds;

		// Check rotation magnitude. If too large, do not calculate error, but set P part to 0 - update PID with zeroes.
		fTemp = vectorf_getNorm(&data->_gyro.vector);

		if(fTemp < data->maxGyroErrorUpdateRate)
		{
			// Check if we update gains
			if(data->sFlag.bits.FLAG_FAST_ROTATION)
			{
				data->sFlag.bits.FLAG_FAST_ROTATION = 0;
				// Update gyro gains
				// Store error used
				data->_gyro.gyroGainError.x = error.x;
				data->_gyro.gyroGainError.y = error.y;
				data->_gyro.gyroGainError.z = error.z;
				// Use calculated error to determine if gain is off
				// X axis was rotating fast?
				if(data->sFlag.bits.SFLAG_X_FAST_ROTATING)
				{
					data->sFlag.bits.SFLAG_X_FAST_ROTATING = 0;
					// Check X axis
					if(data->sFlag.bits.SFLAG_X_ROTATION_DIRECTION)
					{
						// X rotation was positive
						// Update data->_gyro.gyroRateXP
						data->_gyro.gyroRateXP -= error.x * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
					else
					{
						data->_gyro.gyroRateXN += error.x * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
				}
				// Check Y axis
				// X axis was rotating fast?
				if(data->sFlag.bits.SFLAG_Y_FAST_ROTATING)
				{
					data->sFlag.bits.SFLAG_Y_FAST_ROTATING = 0;
					if(data->sFlag.bits.SFLAG_Y_ROTATION_DIRECTION)
					{
						// Y rotation was positive
						data->_gyro.gyroRateYP -= error.y * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
					else
					{
						data->_gyro.gyroRateYN += error.y * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
				}
				// Check Z axis
				// X axis was rotating fast?
				if(data->sFlag.bits.SFLAG_Z_FAST_ROTATING)
				{
					data->sFlag.bits.SFLAG_Z_FAST_ROTATING = 0;
					if(data->sFlag.bits.SFLAG_Z_ROTATION_DIRECTION)
					{
						// Z rotation was positive
						data->_gyro.gyroRateZP -= error.z * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
					else
					{
						data->_gyro.gyroRateZN += error.z * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
				}
				//status = math_PID3(&error, dt, &data->_gyroGainPID);
			}
			// Check update interval
			data->_gyroErrorUpdateCount ++;
			if(data->_gyroErrorUpdateCount > data->PARAMETERS.gyroErrorUpdateInterval)
			{
				data->_gyroErrorUpdateCount = 0;
				// Check when to update I error
				data->_gyroIErrorUpdateCount ++;
				if(data->_gyroIErrorUpdateCount > data->PARAMETERS.gyroIErrorUpdateInterval)
				{
					// Update gyro I drift error
					data->_gyroIErrorUpdateCount = 0;
				}
				else
				{
					// If dt is 0, so is I error update
					dt = 0;
				}
				// Update PID
				status = math_PID3(&error, dt, &data->_gyroErrorPID);
			}
			else
			{
				// If error ~= 0, update PID with 0
				fTemp = vectorf_getNorm(&error);
				if(ERROR_IS_SMALL > fTemp)
				{
					// Update PID with error = 0
					error.x = 0;
					error.y = 0;
					error.z = 0;
					status = math_PID3(&error, 0, &data->_gyroErrorPID);
				}
			}
		}
		else
		{
			// If rotating fast, mark update gyro gains
			if(fTemp > data->gyroFastRotation)
			{
				data->sFlag.bits.FLAG_FAST_ROTATION = 1;
				// Mark rotation directions
				if(0 < data->_gyro.vector.x)
				{
					data->sFlag.bits.SFLAG_X_ROTATION_DIRECTION = 1;
					// Rotating fast?
					if(GYRO_FAST_ROTATION < data->_gyro.vector.x)
					{
						data->sFlag.bits.SFLAG_X_FAST_ROTATING = 1;
					}
				}
				else
				{
					data->sFlag.bits.SFLAG_X_ROTATION_DIRECTION = 0;
					// Rotating fast?
					if(-GYRO_FAST_ROTATION > data->_gyro.vector.x)
					{
						data->sFlag.bits.SFLAG_X_FAST_ROTATING = 1;
					}
				}
				if(0 < data->_gyro.vector.y)
				{
					data->sFlag.bits.SFLAG_Y_ROTATION_DIRECTION = 1;
					// Rotating fast?
					if(GYRO_FAST_ROTATION < data->_gyro.vector.y)
					{
						data->sFlag.bits.SFLAG_Y_FAST_ROTATING = 1;
					}
				}
				else
				{
					data->sFlag.bits.SFLAG_Y_ROTATION_DIRECTION = 0;
					// Rotating fast?
					if(-GYRO_FAST_ROTATION > data->_gyro.vector.y)
					{
						data->sFlag.bits.SFLAG_Y_FAST_ROTATING = 1;
					}
				}
				if(0 < data->_gyro.vector.z)
				{
					data->sFlag.bits.SFLAG_Z_ROTATION_DIRECTION = 1;
					// Rotating fast?
					if(GYRO_FAST_ROTATION < data->_gyro.vector.z)
					{
						data->sFlag.bits.SFLAG_Z_FAST_ROTATING = 1;
					}
				}
				else
				{
					data->sFlag.bits.SFLAG_Z_ROTATION_DIRECTION = 0;
					// Rotating fast?
					if(-GYRO_FAST_ROTATION > data->_gyro.vector.z)
					{
						data->sFlag.bits.SFLAG_Z_FAST_ROTATING = 1;
					}
				}

			}
			// Update drift PID with error = 0
			error.x = 0;
			error.y = 0;
			error.z = 0;
			status = math_PID3(&error, dt, &data->_gyroErrorPID);
		}


	return SUCCESS;
}

// Function updates rotation matrix from gyro data. At end it renormalizes and reorthogonalizes matrix.
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data)
{
	ErrorStatus status;
	status = SUCCESS;

	//Vectorf omega = vectorf_init(0);						// Rotation value
	Matrixf updateMatrix;									// Update matrix
	Matrixf newMatrix;										// Place to store new DCM matrix
	float32_t dt = 0;


	// Calculate delta time
	dt = (float32_t)data->_gyro.deltaTime;
	dt = dt * data->PARAMETERS.systimeToSeconds;

	data->integrationTime = dt;


	// Calculate rotation angle
	// Is gyro value * delta time in seconds
	status = vectorf_scalarProduct(&data->_gyro.vector, dt, &data->updateRotation);
	// Use filtered values
	//status = vectorf_scalarProduct(&data->_gyro.kFilteredVector, dt, &data->updateRotation);

	// If rotation angle above limit, update rotation matrix
	//if(data->PARAMETERS.minRotation < vectorf_getNorm(&data->updateRotation))
	{
		// Store update vector
		//vectorf_copy(&omega, &data->updateRotation);
		// Generate update matrix
		status = fusion_generateUpdateMatrix(&data->updateRotation, &updateMatrix);
		// Multiply DCM and update matrix
		status = matrix3_MatrixMultiply(&data->_fusion_DCM, &updateMatrix, &newMatrix);

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

