/*
 * sensor_macros.h
 *
 *  Created on: 18. mar. 2014
 *      Author: Jure
 */

#ifndef SENSOR_MACROS_H_
#define SENSOR_MACROS_H_

#define SENSOR_SYSTIME_TO_SECONDS				0.00001f	// Convert systemTime to seconds, 1 ms timer, time is stored as 1/100 ms, so divide by 100000.
#define SENSOR_MIN_ROT_ERROR					0.000f
#define SENSOR_MIN_ROTATION						0.000f
#define SENSOR_MIN_GPS_SPEED					2.0f
#define GYRO_ERROR_UPDATE_INTERVAL				10			// Update PID every n cycles
#define GYRO_I_UPDATE_INTERVAL					10			// When to update error integral
#define GYRO_MAX_ERROR_AMPLITUDE				0.5f		// Maximum gyro amplitude when updating error PID


#endif /* SENSOR_MACROS_H_ */
