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
#include "functions.h"


#define GYRO_DEFAULT_RATE					0.030517578125f			// 1000/32768 -> deg/sec
#define GYRO_DEG_TO_RAD						0.017453292519f			// 1 deg/sec is this in rad/sec
#define GYRO_RAD_TO_DEG						57.295779513082f		// 1 rad/sec is this in deg/sec

// Init data structure
ErrorStatus gyro_initDataStructure(GyroData *data)
{
	data->dataTime = getSystemTime();
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
	data->gyroRate = GYRO_DEFAULT_RATE;// * GYRO_DEG_TO_RAD;
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
	data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;
	data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyro.gyroRate * GYRO_DEG_TO_RAD;

	// Filter result
	data->_gyro.vectorKFiltered.x = Kalman_Update(&data->_gyro.kFilter.X, data->_gyro.vectorRaw.x);
	data->_gyro.vectorKFiltered.y = Kalman_Update(&data->_gyro.kFilter.Y, data->_gyro.vectorRaw.y);
	data->_gyro.vectorKFiltered.z = Kalman_Update(&data->_gyro.kFilter.Z, data->_gyro.vectorRaw.z);

	// Copy filtered data for further
	vectorf_copy(&data->_gyro.vectorKFiltered, &data->_gyro.vector);


	// Set gyros to 0 - perfect sensors
/*
	data->_gyro.vector.x = 0;
	data->_gyro.vector.y = 0;
	data->_gyro.vector.z = 0;
*/
	// Remove offsets
/*
	data->_gyro.vector.x -= data->_gyro.offsets.x;
	data->_gyro.vector.y -= data->_gyro.offsets.y;
	data->_gyro.vector.z -= data->_gyro.offsets.z;
*/
	// Remove drift error
	// Drift error is calculated in different .c/.h file

	data->_gyro.vector.x -= data->_gyroErrorPID.x.s;
	data->_gyro.vector.y -= data->_gyroErrorPID.y.s;
	data->_gyro.vector.z -= data->_gyroErrorPID.z.s;

	// Calculate time difference
	deltaTime = dataTime - data->_gyro.dataTime;
	// Do checks on time passed...

	data->_gyro.dataTime = dataTime;
	data->_gyro.deltaTime = deltaTime;

	data->_gyro.fDeltaTime = (float32_t)data->_gyro.deltaTime * data->PARAMETERS.systimeToSeconds;

	return SUCCESS;
}
