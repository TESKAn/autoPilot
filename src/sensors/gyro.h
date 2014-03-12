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


ErrorStatus gyro_initDataStructure(GyroData *data);
ErrorStatus gyro_update(GyroData *data, int16_t *rawData, uint32_t dataTime);

#endif /* GYRO_H_ */
