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


ErrorStatus fusion_init(FUSION_CORE *coreData);
ErrorStatus fusion_dataUpdate(void *data, FUSION_SENSORDATA *sensorData);
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix);
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data);

#endif /* SENSORS_FUSION_H_ */
