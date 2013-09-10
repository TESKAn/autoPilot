/*
 * gps.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */

#ifndef GPS_H_
#define GPS_H_

typedef struct
{
	uint8_t valid;
	Vectorf speed3D;
	float32_t heading;

}__attribute__((aligned(4),packed)) GPSData;

extern GPSData _GPSData;

ErrorStatus gps_initData();

#endif /* GPS_H_ */
