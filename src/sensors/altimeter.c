/*
 * altimeter.c
 *
 *  Created on: Sep 1, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "altimeter.h"

// Init data structure
ErrorStatus altimeter_initDataStructure(AltimeterData *data)
{
	ErrorStatus status = ERROR;

	data->dataTime = getSystemTime();
	data->deltaTime = 0;
	data->pressure = 0;
	data->pressure_frac = 0;
	data->temperature = 0;
	data->valid = 1;

	status = SUCCESS;

	return status;
}

// Update altimeter reading
ErrorStatus altimeter_update(AltimeterData *data, uint32_t rawData_P, uint16_t rawData_T, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	uint16_t temp = 0;
	// Update pressure
	data->pressure_frac = (rawData_P >> 4) & 0x03;
	data->pressure_frac = data->pressure_frac * 25;	// 2 bits = fractions of Pa
	data->pressure = (rawData_P >> 6) & 0x3FFFF;	// 18 bits = pressure in Pa
	// Update temperature
	temp = rawData_T >> 8;
	data->temperature = (float32_t)((rawData_T >> 4) & 0x0F);
	data->temperature = data->temperature * 0.0666f;
	data->temperature += (float32_t) temp;
	// Update delta time and time
	data->deltaTime = dataTime - data->dataTime;
	data->dataTime = dataTime;

	success = SUCCESS;

	return success;
}
