/*
 * gyro.c
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "sensor_typedefs.h"
#include "kalman.h"
#include "gyro.h"


#define GYRO_DEFAULT_RATE					0.0177772//0.030517578125f			// 500/32768 -> deg/sec
#define GYRO_DEG_TO_RAD						0.017453292519f			// 1 deg/sec is this in rad/sec
#define GYRO_RAD_TO_DEG						57.295779513082f		// 1 rad/sec is this in deg/sec

// Init data structure
ErrorStatus gyro_initDataStructure(GyroData *data, uint32_t time)
{
	data->dataTime = time;
	data->deltaTime = 0;

	data->offset = vectori16_init(0);

	data->vfGyroScale.x = GYRO_DEFAULT_RATE;
	data->vfGyroScale.y = -GYRO_DEFAULT_RATE;
	data->vfGyroScale.z = GYRO_DEFAULT_RATE;

	data->vector = vectorf_init(0);

	data->vectorRaw = vectorf_init(0);

	data->offsets.x = 0.007f;
	data->offsets.y = 0.011f;
	data->offsets.z = -0.021f;

	data->sensorTemperature = 0;
	data->valid = 1;

	// Setup kalman filter
	Kalman3_Init(&data->kFilter, 0.022f, 0.617f);

	return SUCCESS;
}
