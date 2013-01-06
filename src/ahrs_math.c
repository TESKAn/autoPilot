/*
 * ahrs_math.c
 *
 *  Created on: Jan 1, 2013
 *      Author: Jure
 */

#include "allinclude.h"

arm_status ahrs_updateVectorPID(PI3Data* PID, vectorData * errorVector)
{
	// Update PID x
	PID->Ix = PID->Ix + errorVector->vector.pData[VECT_X];
	// Limit I part
	PID->Ix = ahrs_limitFloat(PID->Ix, PID->maxIx, PID->minIx);
	PID->Rx = (errorVector->vector.pData[VECT_X] * PID->Kpx) + (PID->Ix * PID->Kix);

	// Update PID y
	PID->Iy = PID->Iy + errorVector->vector.pData[VECT_Y];
	// Limit I part
	PID->Iy = ahrs_limitFloat(PID->Iy, PID->maxIy, PID->minIy);
	PID->Ry = (errorVector->vector.pData[VECT_Y] * PID->Kpy) + (PID->Iy * PID->Kiy);

	// Update PID z
	PID->Iz = PID->Iz + errorVector->vector.pData[VECT_Z];
	// Limit I part
	PID->Iz = ahrs_limitFloat(PID->Iz, PID->maxIz, PID->minIz);
	PID->Rz = (errorVector->vector.pData[VECT_Z] * PID->Kpz) + (PID->Iz * PID->Kiz);

	return SUCCESS;
}

float32_t ahrs_limitFloat(float32_t number, float32_t max, float32_t min)
{
	if(number > max) return max;
	else if(number < min) return min;
	else return number;
}

void ahrs_update_altitude(void)
{
	ahrs_data.Altitude.currentAltitude = (float)((int16_t)BARO) + (float)BARO_FRAC/10;
	ahrs_data.Altitude.deltaTime = systemTime - ahrs_data.Altitude.dataTime;
	ahrs_data.Altitude.dataTime = systemTime;
	ahrs_data.Altitude.verticalSpeed = (ahrs_data.Altitude.lastAltitude - ahrs_data.Altitude.currentAltitude) / (ahrs_data.Altitude.deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.verticalAcceleration = ahrs_data.Altitude.verticalSpeed / (ahrs_data.Altitude.deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.lastAltitude = ahrs_data.Altitude.currentAltitude;
}

void updateScaledVector(vectorData * vector, uint16_t x, uint16_t y, uint16_t z, float rate)
{
	vector->vector.pData[VECT_X] = (float) ((int16_t)x) * rate;
	vector->vector.pData[VECT_Y] = (float) ((int16_t)y) * rate;
	vector->vector.pData[VECT_Z] = (float) ((int16_t)z) * rate;
	// Store delta time
	vector->deltaTime = sensorAcquisitionTime - vector->dataTime;
	// Store time
	vector->dataTime = sensorAcquisitionTime;
}


void ahrs_vectorDataInit(vectorData * vector, VectType type)
{
	vector->dataTime = systemTime;
	vector->deltaTime = 0;
	vector->type = ROW;

	if(type == ROW)
	{
		/* Assign Number of Rows */
		vector->vector.numRows = 1;
		/* Assign Number of Columns */
		vector->vector.numCols = 3;
	}
	else
	{
		/* Assign Number of Rows */
		vector->vector.numRows = 3;
		/* Assign Number of Columns */
		vector->vector.numCols = 1;
	}
	/* Assign Data pointer */
	vector->vector.pData = vector->vectorData;

	// Set vector to 0
	vector->vector.pData[0] = 0;
	vector->vector.pData[1] = 0;
	vector->vector.pData[2] = 0;
}

void ahrs_vector3qDataInit(vector3qData * vector, VectType type)
{
	vector->dataTime = systemTime;
	vector->deltaTime = 0;
	vector->type = ROW;

	if(type == ROW)
	{
		/* Assign Number of Rows */
		vector->vector.numRows = 1;
		/* Assign Number of Columns */
		vector->vector.numCols = 3;
	}
	else
	{
		/* Assign Number of Rows */
		vector->vector.numRows = 3;
		/* Assign Number of Columns */
		vector->vector.numCols = 1;
	}
	/* Assign Data pointer */
	vector->vector.pData = vector->vectorData;

	// Set vector to 0
	vector->vector.pData[0] = 0;
	vector->vector.pData[1] = 0;
	vector->vector.pData[2] = 0;
}

void ahrs_vectorUpdate(vectorData * vector, float32_t i, float32_t j, float32_t k)
{
	vector->deltaTime = systemTime - vector->dataTime;
	vector->dataTime = systemTime;
	vector->vector.pData[0] = i;
	vector->vector.pData[1] = j;
	vector->vector.pData[2] = k;
}

void ahrs_matrix3by3_init(matrix3by3 * matrix)
{
	matrix->dataTime = systemTime;
	matrix->vector.numCols = 3;
	matrix->vector.numRows = 3;
	matrix->vector.pData = matrix->vectorData;
}

void ahrs_generate_rotationMatrix(matrix3by3 * matrix, float roll, float pitch, float yaw)
{
	matrix->dataTime = systemTime;

	float cp = arm_cos_f32(pitch);
	float sp = arm_sin_f32(pitch);
	float cr = arm_cos_f32(roll);
	float sr = arm_sin_f32(roll);
	float cy = arm_cos_f32(yaw);
	float sy = arm_sin_f32(yaw);

	matrix->vectorData[0] = cp * cy;
	matrix->vectorData[1] = cp * sy;
	matrix->vectorData[2] = -sp;
	matrix->vectorData[3] = (sr * sp * cy) - (cr * sy);
	matrix->vectorData[4] = (sr * sp * sy) + (cr * cy);
	matrix->vectorData[5] = sr * cp;
	matrix->vectorData[6] = (cr * sp * cy) + (sr * sy);;
	matrix->vectorData[7] = (cr * sp * sy) - (sr * cy);
	matrix->vectorData[8] = cr * cp;
}

void ahrs_generate_rotationUpdateMatrix(float32_t x, float32_t y, float32_t z, matrix3by3 * matrix)
{
	matrix->dataTime = systemTime;
	matrix->vectorData[0] = 1;
	matrix->vectorData[1] = z;
	matrix->vectorData[2] = -y;
	matrix->vectorData[3] = -z;
	matrix->vectorData[4] = 1;
	matrix->vectorData[5] = x;
	matrix->vectorData[6] = y;
	matrix->vectorData[7] = -x;
	matrix->vectorData[8] = 1;
}

// Multiply vector by scalar
arm_status ahrs_mult_vector_scalar(vectorData * vector, float32_t scalar)
{
	vector->vector.pData[VECT_X] = vector->vector.pData[VECT_X] * scalar;
	vector->vector.pData[VECT_Y] = vector->vector.pData[VECT_Y] * scalar;
	vector->vector.pData[VECT_Z] = vector->vector.pData[VECT_Z] * scalar;
	return SUCCESS;
}

// Calculate cross product of vectors A and B, store to vector C
arm_status ahrs_vect_cross_product(vectorData * vectorA, vectorData * vectorB, vectorData * vectorC)
{
	// Result X = Ay*Bz - Az*By
	vectorC->vector.pData[VECT_X] = vectorA->vector.pData[VECT_Y]*vectorB->vector.pData[VECT_Z] - vectorA->vector.pData[VECT_Z]*vectorB->vector.pData[VECT_Y];
	// Result Y = Az*Bx - Ax*Bz
	vectorC->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Z]*vectorB->vector.pData[VECT_X] - vectorA->vector.pData[VECT_X]*vectorB->vector.pData[VECT_Z];
	// Result Z = Ax*By - Ay*Bx
	vectorC->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_X]*vectorB->vector.pData[VECT_Y] - vectorA->vector.pData[VECT_Y]*vectorB->vector.pData[VECT_X];
	return SUCCESS;
}

// Multiply matrices A and B into matrix C
arm_status ahrs_mult_matrixes(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC)
{
	// Multiply matrices
	arm_status status = arm_mat_mult_f32(&(matrixA->vector), &(matrixB->vector), &(matrixC->vector));
	if(status == ARM_MATH_SUCCESS)
	{
		// Update time when we multiplied
		matrixC->dataTime = systemTime;
	}
	return status;
}

// Multiply vectorA and matrix into vectorB
arm_status ahrs_mult_vector_matrix(vectorData * vectorA, matrix3by3 * matrix, vectorData * vectorB)
{
	arm_status status;

	// Multiply
	status = arm_mat_mult_f32(&(vectorA->vector), &(matrix->vector), &(vectorB->vector));
	if(status == ARM_MATH_SUCCESS)
	{
		// Update time when we multiplied
		vectorB->dataTime = systemTime;
	}
	return status;
}


