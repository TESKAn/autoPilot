/*
 * sensors_fusion.h
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#ifndef SENSORS_FUSION_H_
#define SENSORS_FUSION_H_

// Includes
#include "sensors_typedefs.h"
#include "gyro.h"
#include "accelerometer.h"
#include "mag.h"
#include "airSpeed.h"
#include "gps.h"
#include "altimeter.h"

typedef struct
{
	// Structs for individual sensors
	AccelerometerData _accelerometer;
	GyroData _gyro;
	MagData _mag;
	airSpeedData _airSpeed;
	AltimeterData _altimeter;

	// DCM matrix
	Matrixf _fusion_DCM;


}FUSION_CORE;

ErrorStatus fusion_init(FUSION_CORE *coreData);
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix);
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data);

#endif /* SENSORS_FUSION_H_ */
