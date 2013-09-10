/*
 * gps.c
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "gps.h"

GPSData _GPSData;

ErrorStatus gps_initData()
{
	ErrorStatus status = ERROR;

	_GPSData.valid = 1;
	_GPSData.speed3D = vectorf_init(0);
	_GPSData.heading = 0;

	status = SUCCESS;

	return status;

}
