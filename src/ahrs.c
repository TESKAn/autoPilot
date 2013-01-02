/*
 * ahrs.c
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */


#include "allinclude.h"

// Variables used
AHRSData ahrs_data;

// Temporary matrices and vectors
matrix3by3 tempMatrix;
matrix3by3 holdMatrix;
vectorData tempVector;

void initAHRSStructure(AHRSData * ahrsStructure)
{
	ahrs_vectorDataInit(&(ahrsStructure->AccVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->GyroVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->MagVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->RollPitchYaw), ROW);
	ahrs_matrix3by3_init(&(ahrsStructure->rotationMatrix));
	ahrsStructure->GPSData.altitude = 0;
	ahrsStructure->GPSData.dataTime = systemTime;
	ahrsStructure->GPSData.dataStartTime = systemTime;
	ahrsStructure->GPSData.dataValid = INVALID;
	ahrsStructure->GPSData.latitude = 0;
	ahrsStructure->GPSData.longitude = 0;
	ahrsStructure->GPSData.speed = 0;
	ahrsStructure->GPSData.trackAngle = 0;
	ahrs_resetRotationMatrix();
	// Try to load values from SD card
	if(loadSettings() == ERROR)
	{
		// There was an error, load default values
		ahrsStructure->accRate = DEFAULT_ACC_RATE;
		ahrsStructure->gyroRate = DEFAULT_GYRO_RATE;
		ahrsStructure->magRate = DEFAULT_MAG_RATE;
	}
}

arm_status updateRotationMatrix(matrix3by3 * rotMatrix, vectorData * rotVector)
{
	arm_status status;
	uint8_t i = 0;
	float dT = 0;
	float dWx = 0;
	float dWy = 0;
	float dWz = 0;
	// Calculate change in time
	dT = (float)(rotVector->deltaTime) * SYSTIME_TOSECONDS;
	// Calculate change in angles
	dWx = rotVector->vector.pData[VECT_X] * DEG_TO_RAD * dT;
	dWy = rotVector->vector.pData[VECT_Y] * DEG_TO_RAD * dT;
	dWz = rotVector->vector.pData[VECT_Z] * DEG_TO_RAD * dT;
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
			rotMatrix->vector.pData[i] = holdMatrix.vector.pData[i];
		}
	}
	return status;
}

void ahrs_resetRotationMatrix(void)
{
	ahrs_data.rotationMatrix.vector.pData[0] = 1;
	ahrs_data.rotationMatrix.vector.pData[1] = 0;
	ahrs_data.rotationMatrix.vector.pData[2] = 0;
	ahrs_data.rotationMatrix.vector.pData[3] = 0;
	ahrs_data.rotationMatrix.vector.pData[4] = 1;
	ahrs_data.rotationMatrix.vector.pData[5] = 0;
	ahrs_data.rotationMatrix.vector.pData[6] = 0;
	ahrs_data.rotationMatrix.vector.pData[7] = 0;
	ahrs_data.rotationMatrix.vector.pData[8] = 1;
}

void ahrs_getAngles(matrix3by3 * rotMatrix, vector3f *vector)
{
	vector->pData[0] = rotMatrix->vector.pData[2];
	vector->pData[1] = rotMatrix->vector.pData[5];
	vector->pData[2] = rotMatrix->vector.pData[0];
}
