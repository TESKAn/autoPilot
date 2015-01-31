/*
 * sensor_macros.h
 *
 *  Created on: 18. mar. 2014
 *      Author: Jure
 */

#ifndef SENSOR_MACROS_H_
#define SENSOR_MACROS_H_

// Sensor flag definitions
#define FLAG_FAST_ROTATION						BIT0
#define SFLAG_X_ROTATION_DIRECTION				BIT1
#define SFLAG_Y_ROTATION_DIRECTION				BIT2
#define SFLAG_Z_ROTATION_DIRECTION				BIT3

#define SENSOR_SYSTIME_TO_SECONDS				0.00001f	// Convert systemTime to seconds, 1 ms timer, time is stored as 1/100 ms, so divide by 100000.
#define SENSOR_MIN_ROT_ERROR					0.000f
#define SENSOR_MIN_ROTATION						0.000f
#define SENSOR_MIN_GPS_SPEED					2.0f
#define ERROR_IS_SMALL							0.01f
#define GYRO_ERROR_UPDATE_INTERVAL				0			// Update PID every n cycles
#define GYRO_I_UPDATE_INTERVAL					100			// When to update error integral
#define GYRO_MAX_ERROR_AMPLITUDE				0.5f		// Not sure if useful
#define GYRO_FAST_ROTATION						0.5f		// When do we consider gyro to be rotating fast
#define GYRO_MAX_ERROR_UPDATE_RATE				0.1f		// Maximum gyro amplitude when updating error PID

// Multiply rotation error with this to get gain adjustment
#define GYRO_GAIN_ADJUSTMENT_FACTOR				0.01f

// PID macros
// Drift compensation
#define GYRO_PID_DRIFT_KP						1.0f
#define GYRO_PID_DRIFT_KI						0.1f
#define GYRO_PID_DRIFT_KD						0.0f
#define GYRO_PID_DRIFT_IMAX						100.0f
#define GYRO_PID_DRIFT_IMIN						-100.0f
#define GYRO_PID_DRIFT_SMAX						100.0f
#define GYRO_PID_DRIFT_SMIN						-100.0f
#define GYRO_PID_DRIFT_EMAX						1.0f
#define GYRO_PID_DRIFT_EMIN						-1.0f

// Gyro gain compensation
// Only use I term
#define GYRO_PID_GAIN_KP						0.0f
#define GYRO_PID_GAIN_KI						0.01f
#define GYRO_PID_GAIN_KD						0.0f
#define GYRO_PID_GAIN_IMAX						100.0f
#define GYRO_PID_GAIN_IMIN						-100.0f
#define GYRO_PID_GAIN_SMAX						100.0f
#define GYRO_PID_GAIN_SMIN						-100.0f
#define GYRO_PID_GAIN_EMAX						1.0f
#define GYRO_PID_GAIN_EMIN						-1.0f





#endif /* SENSOR_MACROS_H_ */
