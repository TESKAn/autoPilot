/*
 * sensor_macros.h
 *
 *  Created on: 18. mar. 2014
 *      Author: Jure
 */

#ifndef SENSOR_MACROS_H_
#define SENSOR_MACROS_H_

#define SENSOR_SYSTIME_TO_SECONDS				0.00001f	// Convert systemTime to seconds, 1 ms timer, time is stored as 1/100 ms, so divide by 100000.
#define SENSOR_MIN_ROT_ERROR					0.0001f
#define SENSOR_MIN_ROTATION						0.0001f
#define SENSOR_MIN_GPS_SPEED					2.0f
#define GYRO_ERROR_UPDATE_INTERVAL				1			// Update PID every n cycles


#endif /* SENSOR_MACROS_H_ */
