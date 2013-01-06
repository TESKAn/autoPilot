/*
 * unitConversions.h
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */

#ifndef UNITCONVERSIONS_H_
#define UNITCONVERSIONS_H_


#define DEFAULT_GYRO_RATE	0.030517578125f		// 1000/32768 -> deg/sec
#define DEFAULT_ACC_RATE	0.000244140625f		// 8/32768 -> g
#define DEFAULT_MAG_RATE	0.000635075720f		// 1,3/2047 -> gauss
#define DEG_TO_RAD	0.017453292519f				// 1 deg/sec is this in rad/sec
#define RAD_TO_DEG	57.295779513082f			// 1 rad/sec is this in deg/sec
#define SYSTIME_TOSECONDS	0.001f				// Convert systemTime to seconds
#define DEFAULT_ROLLPITCHCORRECTIONSCALE	0.5f// Weight of roll pitch correction error
#define DEFAULT_YAWCORRECTIONSCALE	0.5f		// Weight of yaw correction error
// PID defaults
#define DEFAULT_KP			0.1f				// Default Kp value
#define DEFAULT_KI			0.001f				// Default Ki value
#define DEFAULT_MAXI		2000.0f				// Maximum integral value - +2 rad/sec = 2 / 0.001 = 2000
#define DEFAULT_MINI		-2000.0f				// Minimum integral value

#endif /* UNITCONVERSIONS_H_ */
