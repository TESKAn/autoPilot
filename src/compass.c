/*
 * compass.c
 *
 *  Created on: Jan 13, 2013
 *      Author: Jure
 */


#include "allinclude.h"

// Variables section
// Hold rotation matrix for soft iron offsets
matrix3by3 compass_softIronMatrix;
// Hold scale factors
vector3fData compass_softScaleVector;
// Hold hard iron offsets
vector3fData compass_hardIronVector;
// Temp matrix update matrix
matrix3by3 compass_softIronMatrixUpdate;
// Maximum result vector
vector3fData compass_maxMagResult;
// Minimum result vector
vector3fData compass_minMagResult;
// Earth field strength in Gauss
float32_t compass_earthGauss = COMPASS_EARTH_MAG_STRENGTH_GAUSS;
// Result matrix
matrix3by3 compass_resultMatrix;

// Functions section
void compass_initCompassCompensation(void)
{
	math_matrix3by3Init(&compass_softIronMatrix, MATH_YES);
	math_vector3fDataInit(&compass_softScaleVector, ROW);
	math_vector3fDataInit(&compass_hardIronVector, ROW);
	math_matrix3by3Init(&compass_softIronMatrixUpdate, MATH_NO);
	math_vector3fDataInit(&compass_maxMagResult, ROW);
	// Set start max value to ridiculous low
	compass_maxMagResult.vector.pData[VECT_X] = -32000;
	compass_maxMagResult.vector.pData[VECT_Y] = -32000;
	compass_maxMagResult.vector.pData[VECT_Z] = -32000;
	math_vector3fDataInit(&compass_minMagResult, ROW);
	// Set start min value to ridiculous high
	compass_minMagResult.vector.pData[VECT_X] = 32000;
	compass_minMagResult.vector.pData[VECT_Y] = 32000;
	compass_minMagResult.vector.pData[VECT_Z] = 32000;
	math_matrix3by3Init(&compass_resultMatrix, MATH_NO);
}

ErrorStatus compass_checkEnoughSamples()
{
	float32_t temp, wx, wy, wz, i, j, k;
	uint8_t okCount = 0;
	wx = 0;
	wy = 0;
	wz = 0;
	i = 0;
	j = 0;
	k = 0;
	// Check if we have enough data to calculate corrections
	// Calculate X max - min
	temp = compass_maxMagResult.vector.pData[VECT_X] - compass_minMagResult.vector.pData[VECT_X];
	if((temp > compass_earthGauss)&&(temp < COMPASS_EARTH_MAG_STRENGTH_GAUSS_MAX))
	{
		okCount++;
	}
	temp = compass_maxMagResult.vector.pData[VECT_Y] - compass_minMagResult.vector.pData[VECT_Y];
	if((temp > compass_earthGauss)&&(temp < COMPASS_EARTH_MAG_STRENGTH_GAUSS_MAX))
	{
		okCount++;
	}
	temp = compass_maxMagResult.vector.pData[VECT_Z] - compass_minMagResult.vector.pData[VECT_Z];
	if((temp > compass_earthGauss)&&(temp < COMPASS_EARTH_MAG_STRENGTH_GAUSS_MAX))
	{
		okCount++;
	}
	if(okCount == 3)
	{
		// If enough data on all axes, calculate
		// First offset = (max + min) / 2
		compass_hardIronVector.vector.pData[VECT_X] = (compass_maxMagResult.vector.pData[VECT_X] + compass_minMagResult.vector.pData[VECT_X]) / 2;
		compass_hardIronVector.vector.pData[VECT_Y] = (compass_maxMagResult.vector.pData[VECT_Y] + compass_minMagResult.vector.pData[VECT_Y]) / 2;
		compass_hardIronVector.vector.pData[VECT_Z] = (compass_maxMagResult.vector.pData[VECT_Z] + compass_minMagResult.vector.pData[VECT_Z]) / 2;
		// Remove offsets from results
		compass_maxMagResult.vector.pData[VECT_X] = compass_maxMagResult.vector.pData[VECT_X] - compass_hardIronVector.vector.pData[VECT_X];
		compass_maxMagResult.vector.pData[VECT_Y] = compass_maxMagResult.vector.pData[VECT_Y] - compass_hardIronVector.vector.pData[VECT_Y];
		compass_maxMagResult.vector.pData[VECT_Z] = compass_maxMagResult.vector.pData[VECT_Z] - compass_hardIronVector.vector.pData[VECT_Z];

		// Calculate rotation angles

		// Create rotation matrix

		// Update rotation matrix to reach goal angles
		okCount = 1;
		do
		{
			// Increase wx
			if(i < wx)
			{
				i = i + COMPASS_MATRIXUPDATESTEP;
			}
			else
			{
				i = i - COMPASS_MATRIXUPDATESTEP;
			}
			// Increase wy
			if(j < wy)
			{
				j = j + COMPASS_MATRIXUPDATESTEP;
			}
			else
			{
				j = j - COMPASS_MATRIXUPDATESTEP;
			}
			// Increase wz
			if(k < wz)
			{
				k = k + COMPASS_MATRIXUPDATESTEP;
			}
			else
			{
				k = k - COMPASS_MATRIXUPDATESTEP;
			}
			// Populate rotation update matrix
			math_generateRotationUpdateMatrix(i, j, k, &compass_softIronMatrixUpdate);
			// Update matrix
			if(math_multMatrices(&compass_softIronMatrix, &compass_softIronMatrixUpdate, &compass_resultMatrix) == ARM_MATH_SUCCESS)
			{
				math_copyMatrix(&compass_resultMatrix, &compass_softIronMatrix);
			}
			// Check if end
			if((math_float32Abs(i - wx) < COMPASS_MATRIXUPDATESTEP) && (math_float32Abs(j - wy) < COMPASS_MATRIXUPDATESTEP) && (math_float32Abs(k - wz) < COMPASS_MATRIXUPDATESTEP))
			{
				okCount = 0;
				// Populate rotation update matrix
				math_generateRotationUpdateMatrix(wx, wy, wz, &compass_softIronMatrixUpdate);
				// Update matrix
				if(math_multMatrices(&compass_softIronMatrix, &compass_softIronMatrixUpdate, &compass_resultMatrix) == ARM_MATH_SUCCESS)
				{
					math_copyMatrix(&compass_resultMatrix, &compass_softIronMatrix);
				}
			}
		}
		while(okCount == 1);
		// Scale vectors - length = 1




	}

	return SUCCESS;
}

ErrorStatus compass_updateMinMaxVectors()
{
	float32_t temp;
	// Data is located in variables:
	// MAG_X for X
	// MAG_Y for Y
	// MAG_Z for Z
	// Format is int16
	// Multiply with  DEFAULT_MAG_RATE to get result in gauss
	// Earth is 0,25 -> 0.65 Gauss
	// Update X
	temp = (float)((int16_t)MAG_X) * DEFAULT_MAG_RATE;
	if(compass_maxMagResult.vector.pData[VECT_X] < temp)
	{
		compass_maxMagResult.vector.pData[VECT_X] = temp;
	}
	else if(compass_minMagResult.vector.pData[VECT_X] > temp)
	{
		compass_minMagResult.vector.pData[VECT_X] = temp;
	}
	// Update Y
	temp = (float)((int16_t)MAG_Y) * DEFAULT_MAG_RATE;
	if(compass_maxMagResult.vector.pData[VECT_Y] < temp)
	{
		compass_maxMagResult.vector.pData[VECT_Y] = temp;
	}
	else if(compass_minMagResult.vector.pData[VECT_Y] > temp)
	{
		compass_minMagResult.vector.pData[VECT_Y] = temp;
	}
	// Update Z
	temp = (float)((int16_t)MAG_Z) * DEFAULT_MAG_RATE;
	if(compass_maxMagResult.vector.pData[VECT_Z] < temp)
	{
		compass_maxMagResult.vector.pData[VECT_Z] = temp;
	}
	else if(compass_minMagResult.vector.pData[VECT_Z] > temp)
	{
		compass_minMagResult.vector.pData[VECT_Z] = temp;
	}


	return SUCCESS;
}
