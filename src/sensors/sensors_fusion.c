/*
 * sensors_fusion.c
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
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
	data->_gyroError.f32GyroErrorUpdateMaxRate = 3.141f;

	// Init maximum gyro amplitude when updating error
	data->maxGyroErrorAmplitude = GYRO_MAX_ERROR_AMPLITUDE;
	data->gyroFastRotation = GYRO_FAST_ROTATION;
	data->maxGyroErrorUpdateRate = GYRO_MAX_ERROR_UPDATE_RATE;

	// Init PIDs
	// Gyro drift
	math_PIDInit(&data->_gyroErrorPID.x, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&data->_gyroErrorPID.y, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);
	math_PIDInit(&data->_gyroErrorPID.z, GYRO_PID_DRIFT_KP, GYRO_PID_DRIFT_KI, GYRO_PID_DRIFT_KD, GYRO_PID_DRIFT_SMIN, GYRO_PID_DRIFT_SMAX);

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

	// Mark wait for valid sensor signals
	data->ui8DoDCMUpdate = 0;
	// Store initial time
	data->ui32SensorInitTime = time;

	return SUCCESS;
}

ErrorStatus fusion_dataUpdate(FUSION_CORE *data, float32_t f32DeltaTime)
{
	Vectorf updateVector;
	ErrorStatus status = SUCCESS;
	Matrixf newMatrix;
	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;

	if(1 == data->ui8DoDCMUpdate)
	{
		// Update rotation matrix
		//fusion_updateRotationMatrix(data);

		// Calculate delta time
		data->integrationTime = f32DeltaTime;

		//fusionData._gyroError.AccError

		updateVector.x = data->_gyro.vector.x + data->_gyroError.AccError.x;
		updateVector.y = data->_gyro.vector.y + data->_gyroError.AccError.y;
		updateVector.z = data->_gyro.vector.z + data->_gyroError.AccError.z;


		//if(MAG_ERROR_OK)
		{
			// Add mag error
			updateVector.x += data->_gyroError.MagError.x;
			updateVector.y += data->_gyroError.MagError.y;
			updateVector.z += data->_gyroError.MagError.z;
		}

		// Calculate current rotation angle
		// Part 1
		// Is gyro value * delta time in seconds
		// Populate elements of matrix
		data->_fusion_update.a.x = 1.0f;
		data->_fusion_update.a.y = -(updateVector.z * f32DeltaTime);
		data->_fusion_update.a.z = updateVector.y * f32DeltaTime;

		data->_fusion_update.b.x = updateVector.z * f32DeltaTime;
		data->_fusion_update.b.y = 1.0f;
		data->_fusion_update.b.z = -(updateVector.x * f32DeltaTime);

		data->_fusion_update.c.x = -(updateVector.y * f32DeltaTime);
		data->_fusion_update.c.y = updateVector.x * f32DeltaTime;
		data->_fusion_update.c.z = 1.0f;

		// Calculate resulting matrix by multiplying matrices A and B
		// Row A.a, col B.x
		b = data->_fusion_DCM.a.y * data->_fusion_update.b.x;
		c = data->_fusion_DCM.a.z * data->_fusion_update.c.x;
		newMatrix.a.x = data->_fusion_DCM.a.x + b + c;

		a = data->_fusion_DCM.a.x * data->_fusion_update.a.y;
		c = data->_fusion_DCM.a.z * data->_fusion_update.c.y;
		newMatrix.a.y = a + data->_fusion_DCM.a.y + c;

		a = data->_fusion_DCM.a.x * data->_fusion_update.a.z;
		b = data->_fusion_DCM.a.y * data->_fusion_update.b.z;
		newMatrix.a.z = a + b + data->_fusion_DCM.a.z;

		b = data->_fusion_DCM.b.y * data->_fusion_update.b.x;
		c = data->_fusion_DCM.b.z * data->_fusion_update.c.x;
		newMatrix.b.x = data->_fusion_DCM.b.x + b + c;

		a = data->_fusion_DCM.b.x * data->_fusion_update.a.y;
		c = data->_fusion_DCM.b.z * data->_fusion_update.c.y;
		newMatrix.b.y = a + data->_fusion_DCM.b.y + c;

		a = data->_fusion_DCM.b.x * data->_fusion_update.a.z;
		b = data->_fusion_DCM.b.y * data->_fusion_update.b.z;
		newMatrix.b.z = a + b + data->_fusion_DCM.b.z;

		b = data->_fusion_DCM.c.y * data->_fusion_update.b.x;
		c = data->_fusion_DCM.c.z * data->_fusion_update.c.x;
		newMatrix.c.x = data->_fusion_DCM.c.x + b + c;

		a = data->_fusion_DCM.c.x * data->_fusion_update.a.y;
		c = data->_fusion_DCM.c.z * data->_fusion_update.c.y;
		newMatrix.c.y = a + data->_fusion_DCM.c.y + c;

		a = data->_fusion_DCM.c.x * data->_fusion_update.a.z;
		b = data->_fusion_DCM.c.y * data->_fusion_update.b.z;
		newMatrix.c.z = a + b + data->_fusion_DCM.c.z;


		// Renormalize and orthogonalize DCM matrix
		status = matrix3_normalizeOrthogonalizeMatrix(&newMatrix, 0.0001f);

		// Check if matrix is OK and copy data
		if(SUCCESS == status)
		{
			matrix3_copy(&newMatrix, &data->_fusion_DCM);
		}
	}
	else
	{
		// Store gyro vector
		vectorf_add(&data->_gyro.vectorRaw, &data->_gyroError.GyroData, &data->_gyroError.GyroData);
		data->_gyroError.ui8SamplesGyro++;
		// Check time
		if(SENSOR_INVALID_TIME < (uint32_t)(TIM2->CNT))
		{
			// If 1 second has passed since power up,try to init DCM matrix to initial value
			if(SUCCESS == fusion_generateDCM(data))
			{
				// And if successful do matrix update
				data->ui8DoDCMUpdate = 1;
				// Also store current gyro values as initial PID values
				vectorf_scalarDivide(&data->_gyroError.GyroData, (float32_t)data->_gyroError.ui8SamplesGyro, &data->_gyroError.GyroData);
				// Set initial PID values
				math_PIDSet(&data->_gyroErrorPID.x, data->_gyroError.GyroData.x);
				math_PIDSet(&data->_gyroErrorPID.y, data->_gyroError.GyroData.y);
				math_PIDSet(&data->_gyroErrorPID.z, data->_gyroError.GyroData.z);
			}
		}
	}

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
		f32Temp = vectorf_getNorm(&data->_mag.vector);
		if(0.0f != f32Temp)
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
	}



	return status;
}
