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
	uint8_t valid;
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf scale;
	Vectorf Speed_3D;
	Vectorf Speed_3D_Frac;
	float32_t speed_3D_dt;
	Vectorui16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t accRate;
}__attribute__((aligned(4),packed)) AccelerometerData;


ErrorStatus acc_initDataStructure(AccelerometerData *data);
ErrorStatus acc_update(AccelerometerData *data, int16_t *rawData, Matrixf *DCM, uint32_t dataTime);

#endif /* ACCELEROMETER_H_ */
