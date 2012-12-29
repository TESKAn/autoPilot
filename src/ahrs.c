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


void updateScaledVector(vectorData * vector, uint16_t x, uint16_t y, uint16_t z, float rate)
{
	// Store time
	vector->dataTime = sensorAcquisitionTime;
	vector->vector.pData[VECT_X] = (float) x * rate;
	vector->vector.pData[VECT_Y] = (float) y * rate;
	vector->vector.pData[VECT_Z] = (float) z * rate;
}


void ahrs_vectorDataInit(vectorData * vector)
{
	vector->dataTime = 0;
	vector->type = ROW;
	ahrs_vector_init(&(vector->vector), vector->vectorData, vector->type);
	// Set vector to 0
	vector->vector.pData[0] = 0;
	vector->vector.pData[1] = 0;
	vector->vector.pData[2] = 0;
}

void ahrs_vectorUpdate(vectorData * vector, float32_t i, float32_t j, float32_t k)
{
	vector->vector.pData[0] = i;
	vector->vector.pData[1] = j;
	vector->vector.pData[2] = k;
}


// Init vector
void ahrs_vector_init(vector3f * S, float32_t * pData, VectType type)
{
	if(type == ROW)
	{
		/* Assign Number of Rows */
		S->numRows = 1;
		/* Assign Number of Columns */
		S->numCols = 3;
	}
	else
	{
		/* Assign Number of Rows */
		S->numRows = 3;
		/* Assign Number of Columns */
		S->numCols = 1;
	}
	/* Assign Data pointer */
  	S->pData = pData;
}
