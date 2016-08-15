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
#include "ubx.h"
#include "sensor_macros.h"
#include "sensor_typedefs.h"
#include "sensors_fusion.h"
#include "gyro.h"
#include "accelerometer.h"
#include "mag.h"
#include "airSpeed.h"
#include "gps.h"
#include "ubx.h"

#include "altimeter.h"

ErrorStatus fusion_init(FUSION_CORE *data, uint32_t time)
{

	if(ERROR == AirSpeed_initDataStructure(&data->_airSpeed, time))
	{
		return ERROR;
	}
	if(ERROR == altimeter_initDataStructure(&data->_altimeter, time))
	{
		return ERROR;
	}
	if(ERROR == gyro_initDataStructure(&data->_gyro, time))
	{
		return ERROR;
	}

	if(ERROR == mag_initDataStructure(&data->_mag, time))
	{
		return ERROR;
	}

	if(ERROR == acc_initDataStructure(&data->_accelerometer, time))
	{
		return ERROR;
	}

	if(ERROR == gps_initData(&data->_gps, time))
	{
		return ERROR;
	}

	ubx_initData();

	// Init errors
	data->_gyroError.f32SampleWindow = 10.0f;
	data->_gyroError.ui8SampleWindow = 10;
	data->_gyroError.AccelerometerData = vectorf_init(0);
	data->_gyroError.DCMDown = vectorf_init(0);
	data->_gyroError.DCMEast = vectorf_init(0);
	data->_gyroError.DCMNorth = vectorf_init(0);
	data->_gyroError.DCMX = vectorf_init(0);
	data->_gyroError.DCMY = vectorf_init(0);
	data->_gyroError.DCMZ = vectorf_init(0);
	data->_gyroError.GyroData = vectorf_init(0);
	data->_gyroError.MagData = vectorf_init(0);
	data->_gyroError.ui8SamplesAcc = 0;
	data->_gyroError.ui8SamplesMag = 0;
	data->_gyroError.ui8SamplesGyro = 0;
	data->_gyroError.f32GyroErrorUpdateMaxRate = 0.6f;

	// Init maximum gyro amplitude when updating error
	data->maxGyroErrorAmplitude = GYRO_MAX_ERROR_AMPLITUDE;
	data->gyroFastRotation = GYRO_FAST_ROTATION;
	data->maxGyroErrorUpdateRate = GYRO_MAX_ERROR_UPDATE_RATE;

	// Init PIDs
	// Gyro drift
	math_PIDInit(&data->_gyroErrorPID.x, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&data->_gyroErrorPID.y, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&data->_gyroErrorPID.z, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);

	// Gyro gain
	math_PIDInit(&data->_gyroGainPID.x, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);
	math_PIDInit(&data->_gyroGainPID.y, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);
	math_PIDInit(&data->_gyroGainPID.z, GYRO_PID_GAIN_KP, GYRO_PID_GAIN_KI, GYRO_PID_GAIN_KD, GYRO_PID_GAIN_SMIN, GYRO_PID_GAIN_SMAX);



	// Create identity matrix
	matrix3_init(1, &data->_fusion_DCM);

	matrix3_init(1, &data->_GPS_DCM);

	// Init temperature
	data->MPUTemperature = 0;

	// Init system parameters
	data->PARAMETERS.systimeToSeconds = SENSOR_SYSTIME_TO_SECONDS;
	data->PARAMETERS.minRotation = SENSOR_MIN_ROTATION;
	data->PARAMETERS.minRotError = SENSOR_MIN_ROT_ERROR;
	data->PARAMETERS.minGPSSpeed = SENSOR_MIN_GPS_SPEED;
	data->PARAMETERS.gyroErrorUpdateInterval = GYRO_ERROR_UPDATE_INTERVAL;
	data->PARAMETERS.gyroIErrorUpdateInterval = GYRO_I_UPDATE_INTERVAL;


	// Set PID for first run
	fusion_initGyroDriftPID(&fusionData);
	fusion_initGyroGainPID(&fusionData);

	// Mark wait for valid sensor signals
	data->sFlag.bits.SFLAG_DO_DCM_UPDATE = 0;
	// Store initial time
	data->ui32SensorInitTime = time;

	return SUCCESS;
}

ErrorStatus fusion_initGyroDriftPID(FUSION_CORE *data)
{
	// Set initial PID values
	math_PIDSet(&data->_gyroErrorPID.x, 0.045f);
	math_PIDSet(&data->_gyroErrorPID.y, -0.009f);
	math_PIDSet(&data->_gyroErrorPID.z, 0.017f);

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
		// Store gyro vector
		vectorf_add(&data->_gyro.vectorRaw, &data->_gyroError.GyroData, &data->_gyroError.GyroData);
		data->_gyroError.ui8SamplesGyro++;
		// Check time
		if(SENSOR_INVALID_TIME < (time - data->ui32SensorInitTime))
		{
			// If 1 second has passed since power up,try to init DCM matrix to initial value
			if(SUCCESS == fusion_generateDCM(data))
			{
				// And if successful do matrix update
				data->sFlag.bits.SFLAG_DO_DCM_UPDATE = 1;
				// Also store current gyro values as initial PID values
				vectorf_scalarDivide(&data->_gyroError.GyroData, (float32_t)data->_gyroError.ui8SamplesGyro, &data->_gyroError.GyroData);
				// Set initial PID values
				math_PIDSet(&data->_gyroErrorPID.x, data->_gyroError.GyroData.x);
				math_PIDSet(&data->_gyroErrorPID.y, data->_gyroError.GyroData.y);
				math_PIDSet(&data->_gyroErrorPID.z, data->_gyroError.GyroData.z);
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
	data->MPUTemperature = ((float32_t) temperatureData / 340.0f) + 36.53f;
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

	// Check how fast we are rotating. Calculate offset error only if below limit
	fTemp = vectorf_getNorm(&data->_gyro.vector);
	if(data->_gyroError.f32GyroErrorUpdateMaxRate > fTemp)
	{
		// Do only if speed below limit and acceleration is really low
		if((data->PARAMETERS.minGPSSpeed > GPSSpeed)&&(1 == data->_accelerometer.valid))
		{
			// Check that we have only gravity - acc. magnitude is 1 +- 0.05
			fTemp = vectorf_getNorm(&data->_accelerometer.vector);
			if((0.95 < fTemp)&&(1.05 > fTemp))
			{
				// Store errors
				// Store acc reading
				vectorf_add(&data->_accelerometer.vectorNormalized, &data->_gyroError.AccelerometerData, &data->_gyroError.AccelerometerData);
				// Store DCM estimation of acc reading
				vectorf_add(&data->_fusion_DCM.c, &data->_gyroError.DCMDown, &data->_gyroError.DCMDown);
				// Store number of samples
				data->_gyroError.ui8SamplesAcc++;
				// Store mag?
				if(data->_mag.valid)
				{
					vectorf_add(&data->_mag.vector, &data->_gyroError.MagData, &data->_gyroError.MagData);
					vectorf_add(&data->_fusion_DCM.b, &data->_gyroError.DCMEast, &data->_gyroError.DCMEast);
					data->_gyroError.ui8SamplesMag++;
				}
				else
				{
					data->_gyroError.MagData = vectorf_init(0);
					data->_gyroError.DCMEast = vectorf_init(0);
					data->_gyroError.ui8SamplesMag = 0;
				}

				// Enough samples?
				if(data->_gyroError.ui8SampleWindow < data->_gyroError.ui8SamplesAcc)
				{
					// Calculate errors
					// Average data
					// Accelerometer
					fTemp = (float32_t)data->_gyroError.ui8SamplesAcc;
					vectorf_scalarDivide(&data->_gyroError.AccelerometerData, fTemp, &data->_gyroError.AccelerometerData);
					vectorf_scalarDivide(&data->_gyroError.DCMDown, fTemp, &data->_gyroError.DCMDown);
					// Do mag if we have samples
					if(0 != data->_gyroError.ui8SamplesMag)
					{
						// Mag
						fTemp = (float32_t)data->_gyroError.ui8SamplesMag;
						vectorf_scalarDivide(&data->_gyroError.MagData, fTemp, &data->_gyroError.MagData);
						vectorf_scalarDivide(&data->_gyroError.DCMEast, fTemp, &data->_gyroError.DCMEast);
						// Calculate mag error
						// Calculate heading error in body frame
						// Use gravity from accelerometer
						// grav X raw mag = earth Y axis
						// earth Y axis X DCM line b is yaw error in body frame
						// temporaryVector - normalized mag with subtracted offsets
						status = vectorf_normalizeAToB(&data->_gyroError.MagData, &temporaryVector);
						// Get where earth's Y axis is supposed to be
						status = vectorf_crossProduct(&data->_gyroError.AccelerometerData, &temporaryVector, &data->_mag.earthYAxis);
						// Normalize
						vectorf_normalize(&data->_mag.earthYAxis);
						// Get error - rotate DCM B axis to our calculated Y axis
						status = vectorf_crossProduct(&data->_gyroError.DCMEast, &data->_mag.earthYAxis, &error_mag);
					}

					// Calculate gravity error
					status = vectorf_crossProduct(&data->_gyroError.DCMDown, &data->_gyroError.AccelerometerData, &error_acc_gravity);
					// Check if there was calculation error
					// If yes, set error to 0
					if(ERROR == status)
					{
						error_acc_gravity = vectorf_init(0);
					}

					// Store time
					data->_gyroError.ui32DeltaTime =  data->dataTime - data->_gyroError.ui32SamplingTime;
					data->_gyroError.ui32SamplingTime = data->dataTime;

					// Calculate delta time
					dt = (float32_t)data->_gyroError.ui32DeltaTime;
					dt = dt * data->PARAMETERS.systimeToSeconds;

					// Zero
					data->_gyroError.AccelerometerData = vectorf_init(0);
					data->_gyroError.DCMDown = vectorf_init(0);
					data->_gyroError.DCMEast = vectorf_init(0);
					data->_gyroError.DCMNorth = vectorf_init(0);
					data->_gyroError.DCMX = vectorf_init(0);
					data->_gyroError.DCMY = vectorf_init(0);
					data->_gyroError.DCMZ = vectorf_init(0);
					data->_gyroError.MagData = vectorf_init(0);
					data->_gyroError.ui8SamplesAcc = 0;
					data->_gyroError.ui8SamplesMag = 0;

					// Sum up all errors
					status = vectorf_add(&error_acc_gravity, &error_gps_acc, &error);
					status = vectorf_add(&error_mag, &error, &error);
					// Scale error
					//vectorf_scalarProduct(&error, data->_gyro.errorScale, &error);
					// Store errors
					vectorf_copy(&error, &data->_gyro.gyroError);

					// Update PID
					status = math_PID3(&error, dt, &data->_gyroErrorPID);
				}
			}
			else
			{
				error_acc_gravity = vectorf_init(0);
				error_mag = vectorf_init(0);
			}
		}
	}
	else
	{
		data->_gyro.gyroError.x = 0.0f;
		data->_gyro.gyroError.y = 0.0f;
		data->_gyro.gyroError.z = 0.0f;
		// Update PID
		status = math_PID3(&error, 0.01f, &data->_gyroErrorPID);
	}


/*
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
	}
	else
	{
		// If rotating fast, and flag is not set, mark update gyro gains and set hysteresis
		if((fTemp > data->gyroFastRotation)&&(!data->sFlag.bits.FLAG_FAST_ROTATION))
		{
			//data->sFlag.bits.FLAG_FAST_ROTATION = 1;
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
*/

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

	// Calculate current rotation angle
	// Part 1
	// Is gyro value * delta time in seconds
	status = vectorf_scalarProduct(&data->_gyro.vector, dt, &data->updateRotation);

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

	//fusion_generateDCM(data);

	return status;
}

