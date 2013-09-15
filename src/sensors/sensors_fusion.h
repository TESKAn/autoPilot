/*
 * sensors_fusion.h
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#ifndef SENSORS_FUSION_H_
#define SENSORS_FUSION_H_


extern Matrixf _fusion_DCM;

ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix);
ErrorStatus fusion_updateRotationMatrix();

#endif /* SENSORS_FUSION_H_ */