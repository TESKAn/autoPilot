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

#define ACC_DEFAULT_RATE					0.00024414807f			// 8/32768 -> g


// Init accelerometer data structure
ErrorStatus acc_initDataStructure(AccelerometerData *data, uint32_t time)
{
	ErrorStatus status = ERROR;
	Kalman3_Init(&data->kFilter, 0.022f, 0.617f);
	data->dataTime = time;
	data->deltaTime = 0;
	data->offset = vectori16_init(0);
	data->gains = vectorf_init(1);
	data->offsets = vectorf_init(0);
	data->vector = vectorf_init(0);
	data->vectorRaw = vectorf_init(0);
	data->Speed_3D = vectorf_init(0);
	data->Speed_3D_Frac = vectorf_init(0);
	data->speed_3D_dt = 0;
	data->accRate = ACC_DEFAULT_RATE;
	data->sensorTemperature = 0;
	data->valid = 1;
	status = SUCCESS;

	// Init offsets
	data->offsets.x = 0.0f;//0.010282576f;
	data->offsets.y = 0.0f;//0.003120095f;
	data->offsets.z = 0.0f;//-0.137751132f;

	// And scale
	data->gains.x = 1.0f;
	data->gains.y = 1.0f;
	data->gains.z = 1.0f;

	return status;
}

ErrorStatus acc_updateOffsets(AccelerometerData *data)
{
	// Remove offsets for axis value << 1
	data->offsets.x = data->vectorRaw.x;
	data->offsets.y = data->vectorRaw.y;
	// Raw Z should be -1
	data->offsets.z = data->vectorRaw.z + 1;

	return SUCCESS;
}

ErrorStatus acc_updateGains(AccelerometerData *data, int axis)
{
	// Total acceleration should be equal to 1
	// If not, it is due to acc gain error
	// If axis = 0, normalize all
	// Else just one axis
	float32_t temp = 0;
	switch(axis)
	{
		case 0:
		{
			// Calculate vector scale
			temp = vectorf_getNorm(&data->vector);
			temp = sqrtf(temp);
			temp = 1 / temp;
			data->gains.x *= temp;
			data->gains.y *= temp;
			data->gains.z *= temp;
			break;
		}
		case 1:
		{
			// X axis
			temp = 1 / data->vector.x;
			data->gains.x *= temp;
			break;
		}
		case 2:
		{
			// Y axis
			temp = 1 / data->vector.y;
			data->gains.y *= temp;
			break;
		}
		case 3:
		{
			// Z axis
			temp = 1 / data->vector.z;
			data->gains.z *= temp;
			break;
		}
	}

	return SUCCESS;
}

// Update accelerometer speed integration
ErrorStatus acc_updateSpeedCalculation(FUSION_CORE *coreData, uint32_t dataTime)
{
	Vectorf temporaryVector = vectorf_init(0);

	float32_t f32Temp;
	float32_t f32Temp1;
	//int32_t fracCalc = 0;
	// We have current accelerometer data, calculate speed in earth coordinates

	// Update speed integration
	// Transform accelerometer data to earth frame
	matrix3_vectorMultiply(&coreData->_fusion_DCM, &coreData->_accelerometer.vector, &temporaryVector);
	// Remove gravity - check sign
	temporaryVector.z += 1.0f;
	// Calculate 3D speed gained, acceleration * time
	// Calculate time
	f32Temp = (float32_t)coreData->_accelerometer.deltaTime * 0.00001f;
	// Accelerometer data is normalized to 9,81 m/s2
	// Mul by gravity acceleration
	f32Temp1 = f32Temp * 9.81f;
	// Calculate speed change in dt
	vectorf_scalarProduct(&temporaryVector, f32Temp1, &temporaryVector);
	// Integrate speed and time
	vectorf_add(&temporaryVector, &coreData->_accelerometer.Speed_3D, &coreData->_accelerometer.Speed_3D);
	// Time integration can have problems after 2,77 hours because of 32 bit float representation
	coreData->_accelerometer.speed_3D_dt = coreData->_accelerometer.speed_3D_dt + f32Temp;
	return SUCCESS;
}

// Update accelerometer reading
ErrorStatus acc_update(FUSION_CORE *coreData, int16_t *rawData, uint32_t dataTime)
{
	uint32_t deltaTime = 0;

	// Update sensor temperature
	coreData->_accelerometer.sensorTemperature = coreData->MPUTemperature;

	// Update accelerometer reading
	// Store raw data, multiply with - to get gravity - acceleration
	coreData->_accelerometer.vectorRaw.x = (float32_t)rawData[0] * -coreData->_accelerometer.accRate;
	coreData->_accelerometer.vectorRaw.y = (float32_t)rawData[1] * -coreData->_accelerometer.accRate;
	coreData->_accelerometer.vectorRaw.z = (float32_t)rawData[2] * -coreData->_accelerometer.accRate;

	// Filter
	//coreData->_accelerometer.vectorKFiltered.x = Kalman_Update(&coreData->_accelerometer.kFilter.X, coreData->_accelerometer.vectorRaw.x);
	//coreData->_accelerometer.vectorKFiltered.y = Kalman_Update(&coreData->_accelerometer.kFilter.Y, coreData->_accelerometer.vectorRaw.y);
	//coreData->_accelerometer.vectorKFiltered.z = Kalman_Update(&coreData->_accelerometer.kFilter.Z, coreData->_accelerometer.vectorRaw.z);


	// Filter
	coreData->_accelerometer.filterAccum.x += coreData->_accelerometer.vectorRaw.x;
	coreData->_accelerometer.vectorKFiltered.x = coreData->_accelerometer.filterAccum.x / 5.0f;
	coreData->_accelerometer.filterAccum.x -= coreData->_accelerometer.vectorKFiltered.x;

	coreData->_accelerometer.filterAccum.y += coreData->_accelerometer.vectorRaw.y;
	coreData->_accelerometer.vectorKFiltered.y = coreData->_accelerometer.filterAccum.y / 5.0f;
	coreData->_accelerometer.filterAccum.y -= coreData->_accelerometer.vectorKFiltered.y;

	coreData->_accelerometer.filterAccum.z += coreData->_accelerometer.vectorRaw.z;
	coreData->_accelerometer.vectorKFiltered.z = coreData->_accelerometer.filterAccum.z / 5.0f;
	coreData->_accelerometer.filterAccum.z -= coreData->_accelerometer.vectorKFiltered.z;


/*
	coreData->_accelerometer.vectorKFiltered.x = coreData->_accelerometer.vectorRaw.x;
	coreData->_accelerometer.vectorKFiltered.y = coreData->_accelerometer.vectorRaw.y;
	coreData->_accelerometer.vectorKFiltered.z = coreData->_accelerometer.vectorRaw.z;
*/

	// Remove offset
	vectorf_add(&coreData->_accelerometer.vectorKFiltered, &coreData->_accelerometer.offsets, &coreData->_accelerometer.vector);

	// Use gain correction on outputs?
	coreData->_accelerometer.vector.x *= coreData->_accelerometer.gains.x;
	coreData->_accelerometer.vector.y *= coreData->_accelerometer.gains.y;
	coreData->_accelerometer.vector.z *= coreData->_accelerometer.gains.z;

	// Calculate time difference
	deltaTime = dataTime - coreData->_accelerometer.dataTime;

	// Also store normalized vector
	vectorf_normalizeAToB(&coreData->_accelerometer.vector, &coreData->_accelerometer.vectorNormalized);
	// Store time information
	coreData->_accelerometer.dataTime = dataTime;
	coreData->_accelerometer.deltaTime = deltaTime;

	return SUCCESS;
}
