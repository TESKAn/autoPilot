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

AltimeterData _altimeterData;

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
ErrorStatus altimeter_update(uint32_t rawData_P, uint16_t rawData_T, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	uint16_t temp = 0;
	// Update pressure
	_altimeterData.pressure_frac = (rawData_P >> 4) & 0x03;
	_altimeterData.pressure_frac = _altimeterData.pressure_frac * 25;	// 2 bits = fractions of Pa
	_altimeterData.pressure = (rawData_P >> 6) & 0x3FFFF;	// 18 bits = pressure in Pa
	// Update temperature
	temp = rawData_T >> 8;
	_altimeterData.temperature = (float32_t)((rawData_T >> 4) & 0x0F);
	_altimeterData.temperature = _altimeterData.temperature * 0.0666f;
	_altimeterData.temperature += (float32_t) temp;
	// Update delta time and time
	_altimeterData.deltaTime = dataTime - _altimeterData.dataTime;
	_altimeterData.dataTime = dataTime;

	success = SUCCESS;

	return success;
}
