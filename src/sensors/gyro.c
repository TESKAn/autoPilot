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

	// gyro rate in radians
	data->gyroRate = GYRO_DEFAULT_RATE * GYRO_DEG_TO_RAD;
	data->sensorTemperature = 0;
	data->valid = 1;

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
	data->_gyro.vector.x = data->_gyro.vectorRaw.x;
	data->_gyro.vector.y = data->_gyro.vectorRaw.y;
	data->_gyro.vector.z = data->_gyro.vectorRaw.z;

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

	return SUCCESS;
}
