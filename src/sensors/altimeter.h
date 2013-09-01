/*
 * altimeter.h
 *
 *  Created on: Sep 1, 2013
 *      Author: Jure
 */

#ifndef ALTIMETER_H_
#define ALTIMETER_H_

typedef struct
{
	uint8_t valid;
	uint32_t pressure;
	uint32_t pressure_frac;
	float32_t temperature;
	uint32_t dataTime;
	uint32_t deltaTime;
}__attribute__((aligned(4),packed)) AltimeterData;

extern AltimeterData _altimeterData;

ErrorStatus altimeter_initDataStructure();
ErrorStatus altimeter_update(uint32_t rawData_P, uint16_t rawData_T, uint32_t dataTime);

#endif /* ALTIMETER_H_ */
