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
#define DEFAULT_KP						0.5f					// Default Kp value
#define DEFAULT_KI						0.2f					// Default Ki value
#define DEFAULT_MAXERR					1.0f
#define DEFAULT_MINERR					-1.0f
#define DEFAULT_MAXI					5.0f					// Maximum integral value - +2 rad/sec = 2 / 0.001 = 2000
#define DEFAULT_MINI					-5.0f					// Minimum integral value
#define DEFAULT_RMAX					5.0f
#define DEFAULT_RMIN					-5.0f
#define DEFAULT_DISCARD_COUNT			100						//how many samples to discard after boot
#define DEFAULT_MAG_INCLINATION			0.0542506509161570f		// Deviation of magnetic field from true north in radians
#define DEFAULT_MAG_DECLINATION			1,0902441579423135f		// Angle of magnetic field

//#define PI								3.1415926535897932384626433832795f

// Softmag matrix elements
/*
#define SOFTMAG_DEFAULT_RXX				0.00191123174f
#define SOFTMAG_DEFAULT_RYX				-1.33473334e-5f
#define SOFTMAG_DEFAULT_RZX				-6.56999257e-5f
#define SOFTMAG_DEFAULT_RXY				-1.33473334e-5f
#define SOFTMAG_DEFAULT_RYY				0.00211779709f
#define SOFTMAG_DEFAULT_RZY				1.49881559e-5f
#define SOFTMAG_DEFAULT_RXZ				-6.56999257e-5f
#define SOFTMAG_DEFAULT_RYZ				1.49881559e-5f
#define SOFTMAG_DEFAULT_RZZ				0.00189577429f
*/
/*
#define SOFTMAG_DEFAULT_RXX				0.000635f
#define SOFTMAG_DEFAULT_RYX				2.03287907e-20f
#define SOFTMAG_DEFAULT_RZX				-2.79520873e-20f
#define SOFTMAG_DEFAULT_RXY				0.0f
#define SOFTMAG_DEFAULT_RYY				0.000635f
#define SOFTMAG_DEFAULT_RZY				2.03287907e-20f
#define SOFTMAG_DEFAULT_RXZ				-8.21621959e-20f
#define SOFTMAG_DEFAULT_RYZ				6.77626358e-21f
#define SOFTMAG_DEFAULT_RZZ				0.000635f
*/

#define SOFTMAG_DEFAULT_RXX   0.00174733926490326f
#define SOFTMAG_DEFAULT_RYX   9.30946253605834e-06f
#define SOFTMAG_DEFAULT_RZX   1.54827707888659e-06f
#define SOFTMAG_DEFAULT_RXY   9.30946253605834e-06f
#define SOFTMAG_DEFAULT_RYY   0.00199220982756265f
#define SOFTMAG_DEFAULT_RZY   4.40037016213924e-05f
#define SOFTMAG_DEFAULT_RXZ   1.54827707888659e-06f
#define SOFTMAG_DEFAULT_RYZ   4.40037016213924e-05f
#define SOFTMAG_DEFAULT_RZZ   0.00181786358178464f
#define HARDIRON_DEFAULT_X    -498.721881815402f
#define HARDIRON_DEFAULT_Y    453.203466252164f
#define HARDIRON_DEFAULT_Z    -1021.90931409698f
#define SOFTMAG_SCALE         591.226172172661f


#endif /* UNITCONVERSIONS_H_ */
