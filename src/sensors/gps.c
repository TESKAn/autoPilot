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

ErrorStatus gps_initData(GPSData *data)
{
	data->valid = 1;
	data->speed3D = vectorf_init(0);
	data->heading = 0;
	return SUCCESS;

}
