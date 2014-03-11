/*
 * gyro.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#ifndef GYRO_H_
#define GYRO_H_

typedef struct
{
	uint8_t valid;
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf driftError;
	Vectorf scale;
	Vectorui16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t gyroRate;
}__attribute__((aligned(4),packed)) GyroData;

extern GyroData _gyroData;

ErrorStatus gyro_initDataStructure(GyroData *data);
ErrorStatus gyro_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime);

#endif /* GYRO_H_ */
