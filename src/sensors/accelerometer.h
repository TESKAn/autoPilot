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
	float32_t x;
	float32_t y;
	float32_t z;

}__attribute__((aligned(4),packed)) accVectorf;

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;

}__attribute__((aligned(4),packed)) accVectorui16;

typedef struct
{
	accVectorf vector;
	accVectorf vectorRaw;
	accVectorf scale;
	accVectorui16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t accRate;
}__attribute__((aligned(4),packed)) AccelerometerData;

extern AccelerometerData _accData;

ErrorStatus acc_initDataStructure();
ErrorStatus acc_update(uint16_t rawData_x, uint16_t rawData_y, uint16_t rawData_z);

#endif /* ACCELEROMETER_H_ */
