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
#include "math/myMath_matrix3.h"
#include "accelerometer.h"

#define ACC_DEFAULT_RATE					0.000244140625f			// 8/32768 -> g
// Define use scale correction
//#define ACC_USE_SCALE_CORRECTION

AccelerometerData _accData;

// Init accelerometer data structure
ErrorStatus acc_initDataStructure()
{
	ErrorStatus status = ERROR;

	_accData.dataTime = getSystemTime();
	_accData.deltaTime = 0;

	_accData.offset = vectorui16_init(0);

	_accData.scale = vectorf_init(1);

	_accData.vector = vectorf_init(0);

	_accData.vectorRaw = vectorf_init(0);

	_accData.Speed_3D = vectorf_init(0);

	_accData.Speed_3D_Frac = vectorf_init(0);

	_accData.accRate = ACC_DEFAULT_RATE;

	_accData.valid = 1;

	status = SUCCESS;

	return status;
}

// Update accelerometer reading
ErrorStatus acc_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, Matrixf * DCM, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	float32_t result[3];
	int32_t fracCalc = 0;
	uint32_t deltaTime = 0;
	Vectorf temporaryVector = vectorf_init(0);
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
	// Invert to get - gravity - acceleration
	// Gravity is in body frame of reference
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

	// We have current accelerometer data, calculate speed in earth coordinates
	// Transfer acceleration data to earth frame of reference
	matrix3_vectorMultiply(DCM, &_accData.vector, &temporaryVector);
	// Next remove gravity from measurement
	//**********************************************************
	// !!!Check what to do with Z value to get correct result!!!
	//**********************************************************
	temporaryVector.z = temporaryVector.z - 1;
	// Integrate x, y, z acceleration over time to get speed change
	temporaryVector.x = deltaTime * temporaryVector.x;
	temporaryVector.y = deltaTime * temporaryVector.y;
	temporaryVector.z = deltaTime * temporaryVector.z;
	// First, add speed to fractional accumulator
	vectorf_add(&temporaryVector, &_accData.Speed_3D_Frac, &_accData.Speed_3D_Frac);
	// If speed values are over 0,001 m/s, add to main speed variable
	// This is to be able to detect and use accelerations smaller than 0,1 m/s
	if(_accData.Speed_3D_Frac.x > 0.001f)
	{
		result[0] = _accData.Speed_3D_Frac.x * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		_accData.Speed_3D.x += result[0];
		_accData.Speed_3D_Frac.x -= result[0];
	}

	if(_accData.Speed_3D_Frac.y > 0.001f)
	{
		result[0] = _accData.Speed_3D_Frac.y * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		_accData.Speed_3D.y += result[0];
		_accData.Speed_3D_Frac.y -= result[0];
	}

	if(_accData.Speed_3D_Frac.z > 0.001f)
	{
		result[0] = _accData.Speed_3D_Frac.z * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		_accData.Speed_3D.z += result[0];
		_accData.Speed_3D_Frac.z -= result[0];
	}


	success = SUCCESS;
	return success;
}
