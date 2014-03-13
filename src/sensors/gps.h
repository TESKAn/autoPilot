/*
 * gps.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */

#ifndef GPS_H_1
#define GPS_H_1

typedef struct
{
	uint8_t valid;
	Vectorf speed3D;
	float32_t heading;
	// latitude
	float32_t latitude;

}__attribute__((aligned(4),packed)) GPSData;


ErrorStatus gps_initData(GPSData *data);

#endif /* GPS_H_ */
