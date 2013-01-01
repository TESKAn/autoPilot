/*
 * ahrs.c
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */


#include "allinclude.h"

// Variables used
// GPS data - updated in gps.c
GPSTypeDef GPS_Data;
uint32_t GPS_DataReceivedTime = 0;
// Gyro
vectorData GyroVector;
// Acceleration
vectorData AccelVector;
// Magnetometer
vectorData MagnetVector;
// Rotation matrix
matrix3by3 rotationMatrix;
// Temporary matrices and vectors
matrix3by3 tempMatrix;
matrix3by3 holdMatrix;
vectorData tempVector;

arm_status updateRotationMatrix(matrix3by3 * rotMatrix, vectorData * rotVector)
{

	arm_status status;
	uint8_t i = 0;
	float dT = 0;
	float dWx = 0;
	float dWy = 0;
	float dWz = 0;
	// Calculate change in angles
	dT = 10;//(float)(systemTime - rotVector->dataTime);
	dWx = rotVector->vector.pData[VECT_X] * dT;
	dWy = rotVector->vector.pData[VECT_Y] * dT;
	dWz = rotVector->vector.pData[VECT_Z] * dT;
	// Update vector
	ahrs_vectorUpdate(&tempVector, dWx, dWy, dWz);
	// Generate update matrix
	ahrs_generate_rotationUpdateMatrix(&tempVector, &tempMatrix);
	// Update main rot matrix
	status = ahrs_mult_matrixes(rotMatrix, &tempMatrix, &holdMatrix);
	// Copy new matrix
	if(status == ARM_MATH_SUCCESS)
	{
		rotMatrix->dataTime = systemTime;
		for(i=0; i < 9; i++)
		{
			rotMatrix->vectorData[i] = holdMatrix.vectorData[i];
		}
	}
	return status;
}


