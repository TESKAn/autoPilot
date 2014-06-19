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

	data->vector = vectorf_init(0);

	data->vectorRaw = vectorf_init(0);

	data->offsets.x = 0.02f;
	data->offsets.y = 0.015f;
	data->offsets.z = -0.03f;
	// gyro rate in radians
	data->gyroRate = GYRO_DEFAULT_RATE;// * GYRO_DEG_TO_RAD;
	data->sensorTemperature = 0;
	data->valid = 1;

	// Setup kalman filter
	Kalman_Init(&data->kFilter_x, 0.022f, 0.617f);
	Kalman_Init(&data->kFilter_y, 0.022f, 0.617f);
	Kalman_Init(&data->kFilter_z, 0.022f, 0.617f);

	return SUCCESS;
}

// Update gyro reading
ErrorStatus gyro_update(FUSION_CORE *data, int16_t *rawData, uint32_t dataTime)
{
	uint32_t deltaTime = 0;

	// Update sensor temperature
	data->_accelerometer.sensorTemperature = data->MPUTemperature;

	// First store raw reading
	data->_gyro.vectorRaw.x = (float32_t)rawData[0] * data->_gyro.gyroRate;
	data->_gyro.vectorRaw.y = (float32_t)rawData[1] * data->_gyro.gyroRate;
	data->_gyro.vectorRaw.z = (float32_t)rawData[2] * data->_gyro.gyroRate;

	// Remove offset
	/*
	rawData[0] -= data->_gyro.offset.x;
	rawData[1] -= data->_gyro.offset.y;
	rawData[2] -= data->_gyro.offset.z;
	 */
	// Calculate rate in rad/sec
	data->_gyro.vector.x = data->_gyro.vectorRaw.x * GYRO_DEG_TO_RAD;
	data->_gyro.vector.y = data->_gyro.vectorRaw.y * GYRO_DEG_TO_RAD;
	data->_gyro.vector.z = data->_gyro.vectorRaw.z * GYRO_DEG_TO_RAD;
	// Filter result
	data->_gyro.vector.x = Kalman_Update(&data->_gyro.kFilter_x, data->_gyro.vector.x);
	data->_gyro.vector.y = Kalman_Update(&data->_gyro.kFilter_y, data->_gyro.vector.y);
	data->_gyro.vector.z = Kalman_Update(&data->_gyro.kFilter_z, data->_gyro.vector.z);

	// Copy filtered data
	vectorf_copy(&data->_gyro.vector, &data->_gyro.kFilteredVector);

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

	// Put through kalman filter
	/*
	data->_gyro.kFilteredVector.x = Kalman_Update(&data->_gyro.kFilter_x, data->_gyro.vector.x);
	data->_gyro.kFilteredVector.y = Kalman_Update(&data->_gyro.kFilter_y, data->_gyro.vector.y);
	data->_gyro.kFilteredVector.z = Kalman_Update(&data->_gyro.kFilter_z, data->_gyro.vector.z);
*/
	// Calculate time difference
	deltaTime = dataTime - data->_gyro.dataTime;
	// Do checks on time passed...

	data->_gyro.dataTime = dataTime;
	data->_gyro.deltaTime = deltaTime;

	return SUCCESS;
}
