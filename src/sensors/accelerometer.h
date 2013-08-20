/*
 * accelerometer.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

typedef struct
{
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf scale;
	Vectorui16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t accRate;
}__attribute__((aligned(4),packed)) AccelerometerData;

extern AccelerometerData _accData;

ErrorStatus acc_initDataStructure();
ErrorStatus acc_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime);

#endif /* ACCELEROMETER_H_ */
