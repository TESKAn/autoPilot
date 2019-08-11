/*
 * mag.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "sensor_typedefs.h"
#include "mag.h"

//#define MAG_DEFAULT_RATE					0.000635075720f			// 1,3/2047 -> gauss, result is in gauss	0,00917431192660550458715596330275
#//define MAG_DEFAULT_RATE					0.000917431192660550458715596330275f			// 1,3/2047 -> gauss, result is in gauss	0,00917431192660550458715596330275

#define MAG_DEFAULT_RATE					0.00012207403790398876918851283303323f
#define MAG_DEF_OFFSET_X					-0.269852f
#define MAG_DEF_OFFSET_Y					0.423706f
#define MAG_DEF_OFFSET_Z					0.058639f

// Init data structure
ErrorStatus mag_initDataStructure(MagData *data, uint32_t time)
{
	ErrorStatus success = ERROR;
	Kalman3_Init(&data->kFilter, 0.022f, 0.617f);
	data->heading = 0.0f;
	data->dataTime = time;
	data->deltaTime = 0;
	data->hardIron = vectorf_init(0);
	data->vfMagRate.x = -MAG_DEFAULT_RATE;
	data->vfMagRate.y = -MAG_DEFAULT_RATE;
	data->vfMagRate.z = MAG_DEFAULT_RATE;
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
	data->magOffsetNullGain = 0.01f;
	data->vectorRaw = vectorf_init(0);
	data->calcVector = vectorf_init(0);
	data->valid = 1;

	// Set default offsets
	data->offset.x = MAG_DEF_OFFSET_X;
	data->offset.y = MAG_DEF_OFFSET_Y;
	data->offset.z = MAG_DEF_OFFSET_Z;

	// Setup offset and soft iron matrix

	data->offset.x = -0.047097f;
	data->offset.y = -0.151491f;
	data->offset.z = 0.019625f;


	data->softIron.a.x = 2.362166f;
	data->softIron.a.y = 0.069429f;
	data->softIron.a.z = 0.016196f;
	data->softIron.b.x = 0.069429f;
	data->softIron.b.y = 2.318745f;
	data->softIron.b.z = 0.011444f;
	data->softIron.c.x = 0.016196f;
	data->softIron.c.y = 0.011444f;
	data->softIron.c.z = 2.369234f;


	success = SUCCESS;

	return success;
}
