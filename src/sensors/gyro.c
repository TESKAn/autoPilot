/*
 * gyro.c
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "sensor_typedefs.h"
#include "kalman.h"
#include "gyro.h"


#define GYRO_DEFAULT_RATE					0.0152587890625f//0.030517578125f			// 500/32768 -> deg/sec
#define GYRO_DEG_TO_RAD						0.017453292519f			// 1 deg/sec is this in rad/sec
#define GYRO_RAD_TO_DEG						57.295779513082f		// 1 rad/sec is this in deg/sec

// Init data structure
ErrorStatus gyro_initDataStructure(GyroData *data, uint32_t time)
{
	data->dataTime = time;
	data->deltaTime = 0;

	data->offset = vectori16_init(0);

	data->scale = vectorf_init(1);

	// How much to scale rotation error
	data->errorScale = 0.1;

	data->vector = vectorf_init(0);

	data->vectorRaw = vectorf_init(0);

	data->offsets.x = 0.01805281f;
	data->offsets.y = 0.016643153f;
	data->offsets.z = -0.034478845f;
	// gyro rate in radians
	data->gyroRate = GYRO_DEFAULT_RATE;
	data->gyroRateXP = GYRO_DEFAULT_RATE;
	data->gyroRateXN = GYRO_DEFAULT_RATE;
	data->gyroRateYP = GYRO_DEFAULT_RATE;
	data->gyroRateYN = GYRO_DEFAULT_RATE;
	data->gyroRateZP = GYRO_DEFAULT_RATE;
	data->gyroRateZN = GYRO_DEFAULT_RATE;
	data->sensorTemperature = 0;
	data->valid = 1;

	// Setup kalman filter
	Kalman3_Init(&data->kFilter, 0.022f, 0.617f);

	return SUCCESS;
}

// Update gyro reading
ErrorStatus gyro_update(FUSION_CORE *data, int16_t *rawData, uint32_t dataTime)
{
	uint32_t deltaTime = 0;

	// Update sensor temperature
	data->_accelerometer.sensorTemperature = data->MPUTemperature;

	// First store raw reading
/*
	data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;
	 */
	// Use gain PID result
	/*
	data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyroGainPID.x.s * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyroGainPID.y.s * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyroGainPID.z.s * GYRO_DEG_TO_RAD;
	*/
	// Use different rates depending on direction of rotation
	// X
	if(0 < rawData[0])
	{
		data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyro.gyroRateXP * GYRO_DEG_TO_RAD;
	}
	else
	{
		data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyro.gyroRateXN * GYRO_DEG_TO_RAD;
	}
	// Y
	if(0 < rawData[1])
	{
		data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyro.gyroRateYP * GYRO_DEG_TO_RAD;
	}
	else
	{
		data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyro.gyroRateYN * GYRO_DEG_TO_RAD;
	}
	// Z
	if(0 < rawData[2])
	{
		data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyro.gyroRateZP * GYRO_DEG_TO_RAD;
	}
	else
	{
		data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyro.gyroRateZN * GYRO_DEG_TO_RAD;
	}

	vectorf_copy(&data->_gyro.vectorRaw, &data->_gyro.vector);

	// Remove offset
	data->_gyro.vector.x -= 0.045f;//data->_gyroErrorPID.x.s;
	data->_gyro.vector.y -= -0.01f;//data->_gyroErrorPID.y.s;
	data->_gyro.vector.z -= 0.02f;//data->_gyroErrorPID.z.s;
	// Calculate time difference
	deltaTime = dataTime - data->_gyro.dataTime;
	// Do checks on time passed...

	data->_gyro.dataTime = dataTime;
	data->_gyro.deltaTime = deltaTime;

	data->_gyro.fDeltaTime = (float32_t)data->_gyro.deltaTime * data->PARAMETERS.systimeToSeconds;

	return SUCCESS;
}
