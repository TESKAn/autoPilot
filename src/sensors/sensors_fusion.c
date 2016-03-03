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

ErrorStatus fusion_init(FUSION_CORE *coreData, uint32_t time)
{

	if(ERROR == AirSpeed_initDataStructure(&coreData->_airSpeed, time))
	{
		return ERROR;
	}
	if(ERROR == altimeter_initDataStructure(&coreData->_altimeter, time))
	{
		return ERROR;
	}
	if(ERROR == gyro_initDataStructure(&coreData->_gyro, time))
	{
		return ERROR;
	}

	if(ERROR == mag_initDataStructure(&coreData->_mag, time))
	{
		return ERROR;
	}

	if(ERROR == acc_initDataStructure(&coreData->_accelerometer, time))
	{
		return ERROR;
	}

	if(ERROR == gps_initData(&coreData->_gps, time))
	{
		return ERROR;
	}

	// Init maximum gyro amplitude when updating error
	coreData->maxGyroErrorAmplitude = GYRO_MAX_ERROR_AMPLITUDE;
	coreData->gyroFastRotation = GYRO_FAST_ROTATION;
	coreData->maxGyroErrorUpdateRate = GYRO_MAX_ERROR_UPDATE_RATE;

	// Init PIDs
	// Gyro drift
	math_PIDInit(&coreData->_gyroErrorPID.x, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&coreData->_gyroErrorPID.y, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&coreData->_gyroErrorPID.z, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);




	// Gyro gain
	math_PIDInit(&coreData->_gyroGainPID.x, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);
	math_PIDInit(&coreData->_gyroGainPID.y, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);
	math_PIDInit(&coreData->_gyroGainPID.z, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);



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

	// Mark wait for valid sensor signals
	coreData->sFlag.bits.SFLAG_DO_DCM_UPDATE = 0;
	// Store initial time
	coreData->ui32SensorInitTime = time;

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

ErrorStatus fusion_dataUpdate(FUSION_CORE *data, FUSION_SENSORDATA *sensorData, uint32_t time)
{
	// Update temperature
	fusion_calculateMPUTemperature(data, sensorData->data.temperature, sensorData->data.dataTakenTime);
	// Update gyro
	gyro_update(data, (int16_t*)&sensorData->arrays.gyro, sensorData->data.dataTakenTime);
	if(data->sFlag.bits.SFLAG_DO_DCM_UPDATE)
	{
		// Update rotation matrix
		fusion_updateRotationMatrix(data);
	}
	else
	{
		// Check time
		if(SENSOR_INVALID_TIME < (time - data->ui32SensorInitTime))
		{
			// If 1 second has passed since power up,try to init DCM matrix to initial value
			if(SUCCESS == fusion_generateDCM(data))
			{
				// And if successful do matrix update
				data->sFlag.bits.SFLAG_DO_DCM_UPDATE = 1;
			}
		}
	}
	// Update acceleration
	acc_update(data, (int16_t*)&sensorData->arrays.acc, sensorData->data.dataTakenTime);
	// Update speed from acceleration
	acc_updateSpeedCalculation(data, sensorData->data.dataTakenTime);
	// Update mag
	mag_update(data, (int16_t*)&sensorData->arrays.mag, sensorData->data.dataTakenTime);
	// Update altimeter
	altimeter_update(data, sensorData->arrays.pressure.statusPressure, sensorData->arrays.baroTemperatureDegrees, sensorData->arrays.baroTemperatureFrac, sensorData->data.dataTakenTime);

	// Estimate gyro error
	fusion_updateGyroError(data);


	return SUCCESS;
}

// Function calculates temperature from MPU 6000 temperature measurement.
ErrorStatus fusion_calculateMPUTemperature(FUSION_CORE *data, int16_t temperatureData, uint32_t dataTime)
{
	data->MPUTemperature = ((float32_t) temperatureData / 340)+36.53f;
	return SUCCESS;
}

// Function that generates DCM matrix from sensors
ErrorStatus fusion_generateDCM(FUSION_CORE *data)
{
	/*
	 * Generate initial DCM matrix from known vectors.

	A row - earth X axis

	B row - earth Y axis

	C row - earth Z axis

		use gravity vector to get C row of DCM

		use magnetometer reading, cross product of mag reading and C row of DCM gives direction of B row of DCM

		A row is cross product of B row with C row

	Renormalize vectors between calculations.

	Additionally GPS heading can be used instead of magnetometer to get B row. Heading is aligned with body frame X axis so it would have to be converted somehow...
	 */
	float32_t f32Temp = 0.0f;
	ErrorStatus status = ERROR;
	// Check that gravity has length ~1
	f32Temp = vectorf_getNorm(&data->_accelerometer.vector);
	if(((1.0f + DCMGEN_GRAVITY_ACCURACY) > f32Temp) && ((1.0f - DCMGEN_GRAVITY_ACCURACY) < f32Temp))
	{
		// C is gravity
		// Copy
		status = vectorf_copy(&data->_accelerometer.vector, &data->_fusion_DCM.c);
		// Normalize vector
		status = vectorf_normalize(&data->_fusion_DCM.c);
		// Get B as cross product of C row and magnetometer vector
		status = vectorf_crossProduct(&data->_fusion_DCM.c, &data->_mag.vector, &data->_fusion_DCM.b);
		// Normalize vector
		status = vectorf_normalize(&data->_fusion_DCM.b);
		// Get A as cross product of B with C
		status = vectorf_crossProduct(&data->_fusion_DCM.b, &data->_fusion_DCM.c, &data->_fusion_DCM.a);
		// Normalize vector
		status = vectorf_normalize(&data->_fusion_DCM.a);
		status = SUCCESS;
	}



	return status;
}

// Function generates rotation update matrix.
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix, int isIdentity)
{
	ErrorStatus status = ERROR;

	// First create identity matrix
	matrix3_init(isIdentity, updateMatrix);
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
		//Vectorf temporaryVector1 = vectorf_init(0);
		//Vectorf temporaryVector2 = vectorf_init(0);
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

		if((1 ==data->_gps.valid)&&(1 == data->_accelerometer.valid)&&(data->PARAMETERS.minGPSSpeed < GPSSpeed))
		{
			// Use GPS and accelerometer speed data to calculate DCM error in earth frame
			// data->_accelerometer.speed_3D_dt is integration time of accelerometer. Should be time between two GPS readouts.
			// Calculate change in GPS speed
			vectorf_substract(&data->_gps.speed3D, &data->_gps.speed3D_m, &temporaryVector);
			// Compare with acceleration speed
			vectorf_crossProduct(&data->_accelerometer.Speed_3D, &temporaryVector, &error_gps_acc);
			// Reset accelerometer speed change vector
			data->_accelerometer.Speed_3D = vectorf_init(0);
			// Save previous GPS speed.
			vectorf_copy(&data->_gps.speed3D, &data->_gps.speed3D_m);


			// Calculate 1/dt
			/*
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
*/
			// Check if all was calculated OK
			if(ERROR == status)
			{
				// There was error, so reset error vector
				error_gps_acc = vectorf_init(0);
			}

		}
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
				// Check if we have valid magnetometer data and calculate yaw error
				// Do only if we have valid accelerometer data, as we are using that to calculate error
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
					status = vectorf_crossProduct(&data->_fusion_DCM.b, &data->_mag.earthYAxis, &error_mag);
				}
				else
				{
					error_mag = vectorf_init(0);
				}
			}
			else
			{
				error_acc_gravity = vectorf_init(0);
				error_mag = vectorf_init(0);
			}
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
				// Y axis was rotating fast?
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
				// Z axis was rotating fast?
				if(data->sFlag.bits.SFLAG_Z_FAST_ROTATING)
				{
					data->sFlag.bits.SFLAG_Z_FAST_ROTATING = 0;
					if(data->sFlag.bits.SFLAG_Z_ROTATION_DIRECTION)
					{
						// Z rotation was positive
						data->_gyro.gyroRateZP += error.z * GYRO_GAIN_ADJUSTMENT_FACTOR;
					}
					else
					{
						data->_gyro.gyroRateZN -= error.z * GYRO_GAIN_ADJUSTMENT_FACTOR;
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
				//fTemp = vectorf_getNorm(&error);
				//if(ERROR_IS_SMALL > fTemp)
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
			// If rotating fast, and flag is not set, mark update gyro gains and set hysteresis
			if((fTemp > data->gyroFastRotation)&&(!data->sFlag.bits.FLAG_FAST_ROTATION))
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
	//float32_t f32Temp = 0.0f;
	//Vectorf coning = vectorf_init(0);
	Vectorf rotation = vectorf_init(0);
	Vectorf rotationCorrection = vectorf_init(0);
	int vecm0 = 0;;
	int vecm1 = 0;
	int vecm2 = 0;

	// Calculate delta time
	dt = (float32_t)data->_gyro.deltaTime;
	dt = dt * data->PARAMETERS.systimeToSeconds;

	data->integrationTime = dt;

	// Calculate current rotation angle
	// Part 1
	// Is gyro value * delta time in seconds
	//status = vectorf_scalarProduct(&data->_gyro.vector, dt, &data->updateRotation);

	// Update vector
	data->ROTATIONS.Phi0Index++;
	// Get index where to store fresh matrix
	if(2 < data->ROTATIONS.Phi0Index) data->ROTATIONS.Phi0Index = 0;
	vecm0 = data->ROTATIONS.Phi0Index;
	vecm1 = data->ROTATIONS.Phi0Index - 1;
	if(vecm1 < 0) vecm1 += 3;
	vecm2 = data->ROTATIONS.Phi0Index - 2;
	if(vecm2 < 0) vecm2 += 3;

	// Get vector
	// Just rotation, no position correction (I term without P term)
	rotation.x = data->_gyro.vectorRaw.x - data->_gyroErrorPID.x.i;
	rotation.y = data->_gyro.vectorRaw.y - data->_gyroErrorPID.y.i;
	rotation.z = data->_gyro.vectorRaw.z - data->_gyroErrorPID.z.i;

	// 1. rotation update from raw vector - PID I term, offset
	status = vectorf_scalarProduct(&rotation, dt, &data->updateRotation);

	// Store current vector
	status = vectorf_copy(&rotation, &data->ROTATIONS.Wi[vecm0]);

	/*
	// 2.rotation update as result of coning
	// Calculate coning
	// Cross product of current update rotation and previous rotation
	status = vectorf_crossProduct(&data->updateRotation, &data->ROTATIONS.Wi[vecm1], &coning);
	status = vectorf_scalarProduct(&coning, 0.5f, &coning);

	// Sum
	status = vectorf_add(&data->updateRotation, &coning, &data->updateRotation);
*/
	// Add P term correction - orientation
	// Calculate rotation update from PID P term
	rotation.x = -data->_gyroErrorPID.x.p;
	rotation.y = -data->_gyroErrorPID.y.p;
	rotation.z = -data->_gyroErrorPID.z.p;

	status = vectorf_scalarProduct(&rotation, dt, &rotationCorrection);

	/*
	data->updateRotation.x = data->updateRotation.x - data->_gyroErrorPID.x.p;
	data->updateRotation.y = data->updateRotation.y - data->_gyroErrorPID.y.p;
	data->updateRotation.z = data->updateRotation.z - data->_gyroErrorPID.z.p;
*/
	// Add orientation correction term to update rotation
	status = vectorf_add(&data->updateRotation, &rotationCorrection, &data->updateRotation);


	// Generate update matrix
	status = fusion_generateUpdateMatrix(&data->updateRotation, &updateMatrix, 1);

	// Multiply DCM and update matrix
	status = matrix3_MatrixMultiply(&data->_fusion_DCM, &updateMatrix, &newMatrix);

	// Renormalize and orthogonalize DCM matrix
	status = matrix3_normalizeOrthogonalizeMatrix(&newMatrix, param_dcm_max_orth_error);

	// Check if matrix is OK and copy data
	if(SUCCESS == status)
	{
		matrix3_copy(&newMatrix, &data->_fusion_DCM);
	}

	return status;
}

