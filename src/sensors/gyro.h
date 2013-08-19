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
	float32_t x;
	float32_t y;
	float32_t z;

}__attribute__((aligned(4),packed)) gyroVectorf;

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;

}__attribute__((aligned(4),packed)) gyroVectorui16;

typedef struct
{
	gyroVectorf vector;
	gyroVectorf vectorRaw;
	gyroVectorf driftError;
	gyroVectorf scale;
	gyroVectorui16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t gyroRate;
}__attribute__((aligned(4),packed)) GyroData;

extern GyroData _gyroData;

ErrorStatus gyro_initDataStructure();
ErrorStatus gyro_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z, uint32_t dataTime);

#endif /* GYRO_H_ */
