/*
 * sensors_fusion.h
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#ifndef SENSORS_FUSION_H_
#define SENSORS_FUSION_H_

// Includes
#include "math/myMath_typedefs.h"
#include "sensor_typedefs.h"
#include "gps.h"
#include "accelerometer.h"


ErrorStatus fusion_init(FUSION_CORE *data, uint32_t time);
ErrorStatus fusion_initGyroDriftPID(FUSION_CORE *data);
ErrorStatus fusion_initGyroGainPID(FUSION_CORE *data);
ErrorStatus fusion_dataUpdate(FUSION_CORE *data, FUSION_SENSORDATA *sensorData, uint32_t time);
ErrorStatus fusion_calculateMPUTemperature(FUSION_CORE *data, int16_t temperatureData, uint32_t dataTime);
ErrorStatus fusion_generateDCM(FUSION_CORE *data);
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix, int isIdentity);
ErrorStatus fusion_updateGyroError(FUSION_CORE *data);
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data);

#endif /* SENSORS_FUSION_H_ */
