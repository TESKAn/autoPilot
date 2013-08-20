/*
 * mag.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MAG_H_
#define MAG_H_


typedef struct
{
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf hardIron;
	Matrixf softIron;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t magRate;
}__attribute__((aligned(4),packed)) MagData;

extern MagData _magData;

ErrorStatus mag_initDataStructure();
ErrorStatus mag_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime);

#endif /* MAG_H_ */
