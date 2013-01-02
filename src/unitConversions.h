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
#define DEG_TO_RAD	0.017453292519f		// 1 deg/sec is this in rad/sec
#define RAD_TO_DEG	57.295779513082f	// 1 rad/sec is this in deg/sec
#define SYSTIME_TOSECONDS	0.001f		// Convert systemTime to seconds

#endif /* UNITCONVERSIONS_H_ */
