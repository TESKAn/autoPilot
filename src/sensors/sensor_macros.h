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
#define SFLAG_X_FAST_ROTATING					BIT4
#define SFLAG_Y_FAST_ROTATING					BIT5
#define SFLAG_Z_FAST_ROTATING					BIT6
#define SFLAG_DO_DCM_UPDATE						BIT7

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

// DCM generation from sensor data macros
#define DCMGEN_GRAVITY_ACCURACY					0.05f		// Absolute value of accelerometer reading must be within this tolerance

// Multiply rotation error with this to get gain adjustment
#define GYRO_GAIN_ADJUSTMENT_FACTOR				0.00001f

// Time to wait before sensor data is considered OK
// 1 tick is 10 usec, wait for 1 sec, 100000 ticks is 1 sec
#define SENSOR_INVALID_TIME						100000

// PID macros
// Drift compensation
#define GYRO_PID_DRIFT_KP						100.0f	// Kp compensates orientation errors. Larger value gets faster correction.
#define GYRO_PID_DRIFT_KI						0.05f	// Ki compensates gyro drift errors. Can be small as drift is small.
#define GYRO_PID_DRIFT_KD						0.0f
#define GYRO_PID_DRIFT_IMAX						2.0f
#define GYRO_PID_DRIFT_IMIN						-2.0f
#define GYRO_PID_DRIFT_SMAX						1.0f
#define GYRO_PID_DRIFT_SMIN						-1.0f
#define GYRO_PID_DRIFT_EMAX						1.0f
#define GYRO_PID_DRIFT_EMIN						-1.0f

// Gyro gain compensation
// Only use I term
#define GYRO_PID_GAIN_KP						0.0f
#define GYRO_PID_GAIN_KI						0.01f
#define GYRO_PID_GAIN_KD						0.0f
#define GYRO_PID_GAIN_IMAX						1.0f
#define GYRO_PID_GAIN_IMIN						-1.0f
#define GYRO_PID_GAIN_SMAX						1.0f
#define GYRO_PID_GAIN_SMIN						-1.0f
#define GYRO_PID_GAIN_EMAX						1.0f
#define GYRO_PID_GAIN_EMIN						-1.0f





#endif /* SENSOR_MACROS_H_ */
