/*
 * compass.h
 *
 *  Created on: Jan 13, 2013
 *      Author: Jure
 */

#ifndef COMPASS_H_
#define COMPASS_H_

// Macros section
#define COMPASS_EARTH_MAG_STRENGTH_GAUSS		0.48f
#define COMPASS_EARTH_MAG_STRENGTH_GAUSS_MAX	5.0f
#define COMPASS_EARTH_MAG_STRENGTH_GAUSS_MIN	-5.0f
// Matrix update step in rad
#define COMPASS_MATRIXUPDATESTEP				0.02f

// Variables section
// Hold rotation matrix for soft iron offsets
extern matrix3by3 compass_softIronMatrix;
// Hold scale factors
extern vector3fData compass_softScaleVector;
// Hold hard iron offsets
extern vector3fData compass_hardIronVector;
// Temp matrix update matrix
extern matrix3by3 compass_softIronMatrixUpdate;
// Maximum result vector
extern vector3fData compass_maxMagResult;
// Minimum result vector
extern vector3fData compass_minMagResult;
// Earth field strength in Gauss
extern float32_t compass_earthGauss;
// Result matrix
extern matrix3by3 compass_resultMatrix;

// Functions section
void compass_initCompassCompensation(void);
ErrorStatus compass_checkEnoughSamples();
ErrorStatus compass_updateMinMaxVectors();

#endif /* COMPASS_H_ */
