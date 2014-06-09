/*
 * accelerometer.c
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"

#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "sensor_typedefs.h"
#include "accelerometer.h"
#include "functions.h"

#define ACC_DEFAULT_RATE					0.000244140625f			// 8/32768 -> g
// Define use scale correction
//#define ACC_USE_SCALE_CORRECTION


// Init accelerometer data structure
ErrorStatus acc_initDataStructure(AccelerometerData *data)
{
	ErrorStatus status = ERROR;

	data->dataTime = getSystemTime();
	data->deltaTime = 0;
	data->offset = vectori16_init(0);
	data->scale = vectorf_init(1);
	data->vector = vectorf_init(0);
	data->vectorRaw = vectorf_init(0);
	data->Speed_3D = vectorf_init(0);
	data->Speed_3D_Frac = vectorf_init(0);
	data->speed_3D_dt = 0;
	data->accRate = ACC_DEFAULT_RATE;
	data->sensorTemperature = 0;
	data->valid = 1;
	status = SUCCESS;

	return status;
}

// Update accelerometer speed integration
ErrorStatus acc_updateSpeedCalculation(FUSION_CORE *coreData, uint32_t dataTime)
{
	Vectorf temporaryVector = vectorf_init(0);

	float32_t result[3];
	int32_t fracCalc = 0;
	// We have current accelerometer data, calculate speed in earth coordinates

	// Transfer acceleration data to earth frame of reference

	matrix3_vectorMultiply(&coreData->_fusion_DCM, &coreData->_accelerometer.vector, &temporaryVector);
	// Next remove gravity from measurement
	//**********************************************************
	// !!!Check what to do with Z value to get correct result!!!
	//**********************************************************
	temporaryVector.z = temporaryVector.z - 1;
	// Integrate x, y, z acceleration over time to get speed change
	/*
	temporaryVector.x = coreData->_accelerometer.deltaTime * temporaryVector.x;
	temporaryVector.y = coreData->_accelerometer.deltaTime * temporaryVector.y;
	temporaryVector.z = coreData->_accelerometer.deltaTime * temporaryVector.z;
	*/
	vectorf_scalarProduct(&temporaryVector, coreData->_accelerometer.deltaTime, &temporaryVector);
	// First, add speed to fractional accumulator
	vectorf_add(&temporaryVector, &coreData->_accelerometer.Speed_3D_Frac, &coreData->_accelerometer.Speed_3D_Frac);
	// If speed values are over 0,001 m/s, add to main speed variable
	// This is to be able to detect and use accelerations smaller than 0,1 m/s2
	if(coreData->_accelerometer.Speed_3D_Frac.x > 0.001f)
	{
		result[0] = coreData->_accelerometer.Speed_3D_Frac.x * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		coreData->_accelerometer.Speed_3D.x += result[0];
		coreData->_accelerometer.Speed_3D_Frac.x -= result[0];
	}

	if(coreData->_accelerometer.Speed_3D_Frac.y > 0.001f)
	{
		result[0] = coreData->_accelerometer.Speed_3D_Frac.y * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		coreData->_accelerometer.Speed_3D.y += result[0];
		coreData->_accelerometer.Speed_3D_Frac.y -= result[0];
	}

	if(coreData->_accelerometer.Speed_3D_Frac.z > 0.001f)
	{
		result[0] = coreData->_accelerometer.Speed_3D_Frac.z * 1000;
		fracCalc = (int32_t) result[0];
		result[0] = (float32_t)fracCalc;
		result[0] = result[0] / 1000;
		coreData->_accelerometer.Speed_3D.z += result[0];
		coreData->_accelerometer.Speed_3D_Frac.z -= result[0];
	}





	// Update speed integration
	// Transform accelerometer data to earth frame
	matrix3_vectorMultiply(&coreData->_fusion_DCM, &coreData->_accelerometer.vector, &temporaryVector);
	// Calculate 3D speed gained, acceleration * time
	// Do not worry about gravity acceleration, we remove that in GPS error update
	vectorf_scalarProduct(&temporaryVector, coreData->_accelerometer.deltaTime, &temporaryVector);
	// Integrate speed and time
	vectorf_add(&temporaryVector, &coreData->_accelerometer.Speed_3D, &coreData->_accelerometer.Speed_3D);
	// Time integration can have problems after 2,77 hours because of 32 bit float representation
	coreData->_accelerometer.speed_3D_dt = coreData->_accelerometer.speed_3D_dt + (float32_t)coreData->_accelerometer.deltaTime;


	return SUCCESS;
}

// Update accelerometer reading
ErrorStatus acc_update(FUSION_CORE *coreData, int16_t *rawData, uint32_t dataTime)
{
	float32_t result[3];
	uint32_t deltaTime = 0;

	// Update sensor temperature
	coreData->_accelerometer.sensorTemperature = coreData->MPUTemperature;

	// Update accelerometer reading
	// Store raw data
	coreData->_accelerometer.vectorRaw.x = (float32_t)rawData[0] * coreData->_accelerometer.accRate;
	coreData->_accelerometer.vectorRaw.y = (float32_t)rawData[1] * coreData->_accelerometer.accRate;
	coreData->_accelerometer.vectorRaw.z = (float32_t)rawData[2] * coreData->_accelerometer.accRate;
	// Remove offset
	rawData[0] -= coreData->_accelerometer.offset.x;
	rawData[1] -= coreData->_accelerometer.offset.y;
	rawData[2] -= coreData->_accelerometer.offset.z;
	// Scale result to get g's
	result[0] = (float32_t)rawData[0] * coreData->_accelerometer.accRate;
	result[1] = (float32_t)rawData[1] * coreData->_accelerometer.accRate;
	result[2] = (float32_t)rawData[2] * coreData->_accelerometer.accRate;
	// Invert to get - gravity - acceleration
	// Gravity is in body frame of reference
	result[0] = -result[0];
	result[1] = -result[1];
	result[2] = -result[2];

	// Use scale correction?
#ifdef ACC_USE_SCALE_CORRECTION
	result[0] *= coreData->_accelerometer.scale.x;
	result[1] *= coreData->_accelerometer.scale.y;
	result[2] *= coreData->_accelerometer.scale.z;
#endif


	// Calculate time difference
	deltaTime = dataTime - coreData->_accelerometer.dataTime;
	// Do checks on time passed...

	// And store result if all OK
	coreData->_accelerometer.vector.x = result[0];
	coreData->_accelerometer.vector.y = result[1];
	coreData->_accelerometer.vector.z = result[2];
	// Store time information
	coreData->_accelerometer.dataTime = dataTime;
	coreData->_accelerometer.deltaTime = deltaTime;

	return SUCCESS;
}
