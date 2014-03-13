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
	uint8_t valid;
	Vectorf vector;
	Vectorf vecorPrevious;	// Store previous result for use in offset removal
	Vectorf vectorRaw;
	Vectorf offset;			// Computed magnetometer offset
	Vectorf hardIron;
	Matrixf softIron;
	uint32_t dataTime;
	uint32_t deltaTime;		// Store time difference between current and previous sample
	float32_t magRate;
}__attribute__((aligned(4),packed)) MagData;


ErrorStatus mag_initDataStructure(MagData *data);
ErrorStatus mag_update(MagData *data, int16_t *rawData, Matrixf * DCM, uint32_t dataTime);

#endif /* MAG_H_ */
