/*
 * mag.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "sensor_typedefs.h"
#include "mag.h"

#define MAG_DEFAULT_RATE					0.000635075720f			// 1,3/2047 -> gauss

// Init data structure
ErrorStatus mag_initDataStructure(MagData *data)
{
	ErrorStatus success = ERROR;

	data->dataTime = getSystemTime();
	data->deltaTime = 0;
	data->hardIron = vectorf_init(0);
	data->magRate = MAG_DEFAULT_RATE;
	data->sensorTemperature = 0;
	matrix3_init(1, &data->softIron);
	data->vector = vectorf_init(0);
	data->vectorEarthFrame = vectorf_init(0);
	data->vecorPrevious = vectorf_init(0);
	data->offset = vectorf_init(0);
	data->currentMagReading = vectorf_init(0);
	data->previousMagReading = vectorf_init(0);
	data->currentMagnitude = 0;
	data->previousMagnitude = 0;
	data->magOffsetNullGain = 0.5f;
	data->vectorRaw = vectorf_init(0);
	data->calcVector = vectorf_init(0);
	data->valid = 1;
	success = SUCCESS;

	return success;
}

ErrorStatus mag_update(FUSION_CORE *data, int16_t *rawData, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	float32_t temp;

	// Update mag readings
	// Store mag reading and magnitude to previous
	vectorf_copy(&data->_mag.currentMagReading, &data->_mag.previousMagReading);
	data->_mag.previousMagnitude = data->_mag.currentMagnitude;
	// Update current reading
	data->_mag.currentMagReading.x = (float32_t)rawData[0] * data->_mag.magRate;
	data->_mag.currentMagReading.y = (float32_t)rawData[1] * data->_mag.magRate;
	data->_mag.currentMagReading.z = (float32_t)rawData[2] * data->_mag.magRate;
	// Remove offset
	vectorf_substract(&data->_mag.currentMagReading, &data->_mag.offset, &data->_mag.currentMagReading);
	// Get vector difference
	vectorf_substract(&data->_mag.currentMagReading, &data->_mag.previousMagReading, &data->_mag.calcVector);
	// Try to normalize vector
	if(ERROR != vectorf_normalize(&data->_mag.calcVector))
	{
		// Calculate current magnitude
		data->_mag.currentMagnitude = vectorf_getNorm(&data->_mag.currentMagReading);
		// Calculate magnitude difference
		temp = data->_mag.currentMagnitude - data->_mag.previousMagnitude;
		// Multiply by gain
		temp = temp * data->_mag.magOffsetNullGain;
		// Multiply current offset estimate vector
		vectorf_scalarProduct(&data->_mag.calcVector, temp, &data->_mag.calcVector);
		// Add to offset calculation
		vectorf_add(&data->_mag.offset, &data->_mag.calcVector, &data->_mag.offset);
	}
	// Normalize measurement to get mag direction in body frame
	if(ERROR != vectorf_normalizeAToB(&data->_mag.currentMagReading, &data->_mag.vector))
	{
		// Normalization good, continue with calculations
		// Move to earth reference frame
		matrix3_vectorMultiply(&data->_fusion_DCM, &data->_mag.vector, &data->_mag.vectorEarthFrame);
		// Check mag X,Y vector direction with a reference - like GPS reading
	}


	success = SUCCESS;

	return success;
}

