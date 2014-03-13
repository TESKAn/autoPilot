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

	matrix3_init(1, &data->softIron);

	data->vector = vectorf_init(0);

	data->vectorRaw = vectorf_init(0);

	data->valid = 1;

	success = SUCCESS;

	return success;
}

ErrorStatus mag_update(MagData *data, int16_t *rawData, Matrixf * DCM, uint32_t dataTime)
{
	ErrorStatus success = ERROR;

	// Use _fusion_DCM to null offset

	success = SUCCESS;

	return success;
}

