/*
 * gyro.c
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "customTypedefs.h"
#include "math/myMath_vec3.h"
#include "gyro.h"

#define GYRO_DEFAULT_RATE					0.030517578125f			// 1000/32768 -> deg/sec
#define GYRO_DEG_TO_RAD						0.017453292519f			// 1 deg/sec is this in rad/sec
#define GYRO_RAD_TO_DEG						57.295779513082f		// 1 rad/sec is this in deg/sec

GyroData _gyroData;

// Init data structure
ErrorStatus gyro_initDataStructure()
{
	ErrorStatus success = ERROR;

	_gyroData.dataTime = getSystemTime();
	_gyroData.deltaTime = 0;

	_gyroData.offset.x = 0;
	_gyroData.offset.y = 0;
	_gyroData.offset.z = 0;

	_gyroData.scale.x = 1;
	_gyroData.scale.y = 1;
	_gyroData.scale.z = 1;

	_gyroData.vector.x = 0;
	_gyroData.vector.y = 0;
	_gyroData.vector.z = 0;

	_gyroData.vectorRaw.x = 0;
	_gyroData.vectorRaw.y = 0;
	_gyroData.vectorRaw.z = 0;

	_gyroData.driftError.x = 0;
	_gyroData.driftError.y = 0;
	_gyroData.driftError.z = 0;

	// gyro rate in radians
	_gyroData.gyroRate = GYRO_DEFAULT_RATE * GYRO_DEG_TO_RAD;

	success = SUCCESS;

	return success;
}

// Update gyro reading
ErrorStatus gyro_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	float32_t result[3];
	uint32_t deltaTime = 0;

	// First store raw reading
	_gyroData.vectorRaw.x = (float32_t)rawData_x * _gyroData.gyroRate;
	_gyroData.vectorRaw.y = (float32_t)rawData_y * _gyroData.gyroRate;
	_gyroData.vectorRaw.z = (float32_t)rawData_z * _gyroData.gyroRate;

	// Remove offset
	rawData_x -= _gyroData.offset.x;
	rawData_y -= _gyroData.offset.y;
	rawData_z -= _gyroData.offset.z;

	// Calculate
	result[0] = (float32_t)rawData_x * _gyroData.gyroRate;
	result[1] = (float32_t)rawData_y * _gyroData.gyroRate;
	result[2] = (float32_t)rawData_z * _gyroData.gyroRate;

	// Remove drift error
	// Drift error is calculated in different .c/.h file
	result[0] -= _gyroData.driftError.x;
	result[1] -= _gyroData.driftError.y;
	result[2] -= _gyroData.driftError.z;

	// Calculate time difference
	deltaTime = dataTime - _gyroData.dataTime;
	// Do checks on time passed...

	_gyroData.vector.x = result[0];
	_gyroData.vector.y = result[1];
	_gyroData.vector.z = result[2];

	_gyroData.dataTime = dataTime;
	_gyroData.deltaTime = deltaTime;

	success = SUCCESS;

	return success;
}
