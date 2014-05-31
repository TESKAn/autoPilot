/*
 * airSpeed.c
 *
 *  Created on: Aug 18, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "sensor_typedefs.h"
#include "airSpeed.h"

// We need some variables to calculate airspeed
// pitot AD sample, which we have to convert to Pa
// Baro pressure, which is in Pa
// Ambient temperature, which has to be in Kelvins
// AD result is uint16_t


ErrorStatus AirSpeed_initDataStructure(airSpeedData *data)
{
	data->valid = 1;
	data->airSpeed = 0;
	data->dataTime = getSystemTime();
	data->deltaTime = 0;
	return SUCCESS;
}

// Pp is pitot pressure difference
// Pb is baro pressure from altimeter
// T is temperature in degC * 100
ErrorStatus AirSpeed_CalculateAirSpeed(airSpeedData *data, uint16_t Pp, float32_t Pb, uint16_t T)
{
	ErrorStatus status = ERROR;
	// Calculate float temperature
	float32_t temperature = (float32_t)T;
	temperature = temperature / 100;
	// Calculate float pressure difference
	float32_t Ppfloat = (float32_t)Pp;
	// Formula is v = sqrt(462,5446289 * ((Pp*T)/Pb))
	float32_t result = 462.5446289f * Ppfloat * temperature;
	result = result / Pb;
	result = sqrtf(result);
	// Update airspeed data, do some filtering
	data->airSpeed = 0.7f * data->airSpeed + 0.3f * result;
	// Mark time when we calculated speed
	data->dataTime = getSystemTime();

	return status;
}
