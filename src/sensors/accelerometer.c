/*
 * accelerometer.c
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "accelerometer.h"

#define ACC_DEFAULT_RATE					0.000244140625f			// 8/32768 -> g
// Define use scale correction
//#define ACC_USE_SCALE_CORRECTION

AccelerometerData _accData;

// Init accelerometer data structure
ErrorStatus acc_initDataStructure()
{
	ErrorStatus success = ERROR;

	_accData.dataTime = getSystemTime();
	_accData.deltaTime = 0;

	_accData.offset.x = 0;
	_accData.offset.y = 0;
	_accData.offset.z = 0;

	_accData.scale.x = 1;
	_accData.scale.y = 1;
	_accData.scale.z = 1;

	_accData.vector.x = 0;
	_accData.vector.y = 0;
	_accData.vector.z = 0;

	_accData.vectorRaw.x = 0;
	_accData.vectorRaw.y = 0;
	_accData.vectorRaw.z = 0;

	_accData.accRate = ACC_DEFAULT_RATE;

	success = SUCCESS;

	return success;
}

// Update accelerometer reading
ErrorStatus acc_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	float32_t result[3];
	uint32_t deltaTime = 0;
	// Update accelerometer reading
	// Store raw data
	_accData.vectorRaw.x = (float32_t)rawData_x * _accData.accRate;
	_accData.vectorRaw.y = (float32_t)rawData_y * _accData.accRate;
	_accData.vectorRaw.z = (float32_t)rawData_z * _accData.accRate;
	// Remove offset
	rawData_x -= _accData.offset.x;
	rawData_y -= _accData.offset.y;
	rawData_z -= _accData.offset.z;
	// Scale result to get g's
	result[0] = (float32_t)rawData_x * _accData.accRate;
	result[1] = (float32_t)rawData_y * _accData.accRate;
	result[2] = (float32_t)rawData_z * _accData.accRate;
	// Invert to get gravity - acceleration
	result[0] = -result[0];
	result[1] = -result[1];
	result[2] = -result[2];

	// Use scale correction?
#ifdef ACC_USE_SCALE_CORRECTION
	result[0] *= _accData.scale.x;
	result[1] *= _accData.scale.y;
	result[2] *= _accData.scale.z;
#endif


	// Calculate time difference
	deltaTime = dataTime - _accData.dataTime;
	// Do checks on time passed...

	// And store result if all OK
	_accData.vector.x = result[0];
	_accData.vector.y = result[1];
	_accData.vector.z = result[2];
	// Store time information
	_accData.dataTime = dataTime;
	_accData.deltaTime = deltaTime;
	success = SUCCESS;

	return success;
}
