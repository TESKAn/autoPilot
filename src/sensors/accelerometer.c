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


// Init accelerometer data structure
ErrorStatus acc_initDataStructure(AccelerometerData *data)
{
	ErrorStatus status = ERROR;

	data->dataTime = getSystemTime();
	data->deltaTime = 0;
	data->offset = vectorui16_init(0);
	data->scale = vectorf_init(1);
	data->vector = vectorf_init(0);
	data->vectorRaw = vectorf_init(0);
	data->Speed_3D = vectorf_init(0);
	data->Speed_3D_Frac = vectorf_init(0);
	data->speed_3D_dt = 0;
	data->accRate = ACC_DEFAULT_RATE;
	data->valid = 1;
	status = SUCCESS;

	return status;
}

// Update accelerometer reading
ErrorStatus acc_update(AccelerometerData *data, uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, Matrixf * DCM, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	float32_t result[3];
	int32_t fracCalc = 0;
	uint32_t deltaTime = 0;
	Vectorf temporaryVector = vectorf_init(0);
	// Update accelerometer reading
	// Store raw data
	data->vectorRaw.x = (float32_t)rawData_x * data->accRate;
	data->vectorRaw.y = (float32_t)rawData_y * data->accRate;
	data->vectorRaw.z = (float32_t)rawData_z * data->accRate;
	// Remove offset
	rawData_x -= data->offset.x;
	rawData_y -= data->offset.y;
	rawData_z -= data->offset.z;
	// Scale result to get g's
	result[0] = (float32_t)rawData_x * data->accRate;
	result[1] = (float32_t)rawData_y * data->accRate;
	result[2] = (float32_t)rawData_z * data->accRate;
	// Invert to get - gravity - acceleration
	// Gravity is in body frame of reference
	result[0] = -result[0];
	result[1] = -result[1];
	result[2] = -result[2];

	// Use scale correction?
#ifdef ACC_USE_SCALE_CORRECTION
	result[0] *= data->scale.x;
	result[1] *= data->scale.y;
	result[2] *= data->scale.z;
#endif


	// Calculate time difference
	deltaTime = dataTime - data->dataTime;
	// Do checks on time passed...

	// And store result if all OK
	data->vector.x = result[0];
	data->vector.y = result[1];
	data->vector.z = result[2];
	// Store time information
	data->dataTime = dataTime;
	data->deltaTime = deltaTime;

	// We have current accelerometer data, calculate speed in earth coordinates
	// Transfer acceleration data to earth frame of reference
	matrix3_vectorMultiply(DCM, &data->vector, &temporaryVector);
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
	vectorf_add(&temporaryVector, &data->Speed_3D_Frac, &data->Speed_3D_Frac);
	// If speed values are over 0,001 m/s, add to main speed variable
	// This is to be able to detect and use accelerations smaller than 0,1 m/s
	if(data->Speed_3D_Frac.x > 0.001f)
	{
		result[0] = data->Speed_3D_Frac.x * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		data->Speed_3D.x += result[0];
		data->Speed_3D_Frac.x -= result[0];
	}

	if(data->Speed_3D_Frac.y > 0.001f)
	{
		result[0] = data->Speed_3D_Frac.y * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		data->Speed_3D.y += result[0];
		data->Speed_3D_Frac.y -= result[0];
	}

	if(data->Speed_3D_Frac.z > 0.001f)
	{
		result[0] = data->Speed_3D_Frac.z * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		data->Speed_3D.z += result[0];
		data->Speed_3D_Frac.z -= result[0];
	}


	success = SUCCESS;
	return success;
}
