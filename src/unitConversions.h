/*
 * unitConversions.h
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */

#ifndef UNITCONVERSIONS_H_
#define UNITCONVERSIONS_H_


#define DEFAULT_GYRO_RATE					0.030517578125f			// 1000/32768 -> deg/sec
#define DEFAULT_ACC_RATE					0.000244140625f			// 8/32768 -> g
#define DEFAULT_MAG_RATE					0.000635075720f			// 1,3/2047 -> gauss
#define DEG_TO_RAD							0.017453292519f			// 1 deg/sec is this in rad/sec
#define RAD_TO_DEG							57.295779513082f		// 1 rad/sec is this in deg/sec
//#define SYSTIME_TOSECONDS					0.00001f				// Convert systemTime to seconds, 1 ms timer, time is stored as 1/100 ms, so divide by 100000.
#define DEFAULT_ROLLPITCHCORRECTIONSCALE	0.5f					// Weight of roll pitch correction error
#define DEFAULT_YAWCORRECTIONSCALE			0.5f					// Weight of yaw correction error
// PID defaults
#define DEFAULT_KP							0.5f					// Default Kp value
#define DEFAULT_KI							0.2f					// Default Ki value
#define DEFAULT_MAXERR						1.0f
#define DEFAULT_MINERR						-1.0f
#define DEFAULT_MAXI						5.0f					// Maximum integral value - +2 rad/sec = 2 / 0.001 = 2000
#define DEFAULT_MINI						-5.0f					// Minimum integral value
#define DEFAULT_RMAX						5.0f
#define DEFAULT_RMIN						-5.0f
#define DEFAULT_DISCARD_COUNT				100						//how many samples to discard after boot
#define DEFAULT_MAG_INCLINATION				0.0542506509161570f		// Deviation of magnetic field from true north in radians
#define DEFAULT_MAG_DECLINATION				1.0902441579423135f		// Angle of magnetic field

#define DEFAULT_USE_GPS_SPEED				5.0f					// Min speed in m/s to use GPS heading instead of compass


#define DEFAULT_PID_ERROR_THRESHOLD		0.0001f						// PID error threshold - do not update PID below this
#define DEFAULT_MIN_ROTATION_RATE		0.0001f						// Minimal rotation to count as actual rotation

// Softmag matrix elements
#define SOFTMAG_DEFAULT_RXX   0.00179807074567474f
#define SOFTMAG_DEFAULT_RYX   1.02639947492707e-05f
#define SOFTMAG_DEFAULT_RZX   1.68944746965327e-05f
#define SOFTMAG_DEFAULT_RXY   1.02639947492707e-05f
#define SOFTMAG_DEFAULT_RYY   0.00181033707247978f
#define SOFTMAG_DEFAULT_RZY   -5.53886833860487e-06f
#define SOFTMAG_DEFAULT_RXZ   1.68944746965327e-05f
#define SOFTMAG_DEFAULT_RYZ   -5.53886833860487e-06f
#define SOFTMAG_DEFAULT_RZZ   0.00204592378896502f
// Hard mag vector
#define HARDIRON_DEFAULT_X    30.2441876259683f
#define HARDIRON_DEFAULT_Y    -213.891056269455f
#define HARDIRON_DEFAULT_Z    240.079483297234f
#define SOFTMAG_SCALE         591.226172172661f


#endif /* UNITCONVERSIONS_H_ */
