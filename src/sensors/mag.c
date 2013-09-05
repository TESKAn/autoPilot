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
#include "sensors_fusion.h"
#include "mag.h"

#define MAG_DEFAULT_RATE					0.000635075720f			// 1,3/2047 -> gauss

MagData _magData;

// Init data structure
ErrorStatus mag_initDataStructure()
{
	ErrorStatus success = ERROR;

	_magData.dataTime = getSystemTime();
	_magData.deltaTime = 0;

	_magData.hardIron = vectorf_init(0);

	_magData.magRate = MAG_DEFAULT_RATE;

	matrix3_init(1, &_magData.softIron);

	_magData.vector = vectorf_init(0);

	_magData.vectorRaw = vectorf_init(0);

	_magData.valid = 1;

	success = SUCCESS;

	return success;
}

ErrorStatus mag_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, Matrixf * DCM, uint32_t dataTime)
{
	ErrorStatus success = ERROR;

	// Use _fusion_DCM to null offset

	success = SUCCESS;

	return success;
}

