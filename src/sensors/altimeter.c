/*
 * altimeter.c
 *
 *  Created on: Sep 1, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "sensor_typedefs.h"
#include "altimeter.h"

// Init data structure
ErrorStatus altimeter_initDataStructure(AltimeterData *data, uint32_t time)
{

	data->dataTime = time;
	data->deltaTime = 0;
	data->pressure = 0;
	data->altitude = 0;
	data->temperature = 0;
	data->valid = 1;

	return SUCCESS;
}

// Update altimeter reading
ErrorStatus altimeter_update(FUSION_CORE *data, uint32_t rawData_P, int8_t temp_deg, uint8_t temp_frac, uint32_t dataTime)
{
	ErrorStatus success = ERROR;
	uint32_t temp = 0;
	//int16_t iTemp = 0;

	// Check data valid
	if(rawData_P & 0x02000000)
	{
		data->_altimeter.valid = 250;
	}
	else
	{
		data->_altimeter.valid = 0;
	}
	// Extract pressure data

	// First, fraction
	temp = rawData_P;
	temp = temp >> 4;
	temp = temp & 0x000003;
	data->_altimeter.pressure = (float32_t) temp * 0.25f;
	temp = rawData_P >> 6;
	temp = temp & 0x03FFFF;
	data->_altimeter.pressure += temp;

	/*
	// Extract altitude data

	temp = rawData_P;
	temp = temp >> 4;
	temp = temp & 0xF;
	data->_altimeter.altitude = (float32_t)temp / 10;

	temp = (rawData_P >> 8);
	temp = temp & 0xFFFF;
	iTemp = (int16_t)temp;

	data->_altimeter.altitude += (float32_t)iTemp;
	 */
	// Update temperature
	data->_altimeter.temperature = (float32_t)temp_frac;
	data->_altimeter.temperature = data->_altimeter.temperature / 160;
	data->_altimeter.temperature = data->_altimeter.temperature + (float32_t)temp_deg;

	// Calculate altitude
	float A = data->_altimeter.pressure/101325;
	float B = 1/5.25588;
	float C = powf(A,B);
	C = 1 - C;
	C = C /0.0000225577;

	data->_altimeter.altitude = C;

	// Update delta time and time
	data->_altimeter.deltaTime = dataTime - data->_altimeter.dataTime;
	data->_altimeter.dataTime = dataTime;

	success = SUCCESS;

	return success;
}
