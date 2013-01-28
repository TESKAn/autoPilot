/*
 * math_custom.c
 *
 *  Created on: Jan 13, 2013
 *      Author: Jure
 */

#include "allinclude.h"

float32_t math_float32Abs(float32_t x)
{
	if(x < 0) return -x;
	else return x;
}

float32_t math_limitFloat(float32_t number, float32_t max, float32_t min)
{
	if(number > max) return max;
	else if(number < min) return min;
	else return number;
}


void math_vector3fDataInit(vector3fData * vector, VectType type)
{
	vector->dataTime = systemTime;
	vector->deltaTime = 0;
	getFTime(&(vector->fDataTime));
	vector->fDeltaTime = 0;
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
	vector->vector.pData = vector->vector3fData;

	// Set vector to 0
	vector->vector.pData[VECT_X] = 0;
	vector->vector.pData[VECT_Y] = 0;
	vector->vector.pData[VECT_Z] = 0;
}

void math_vector3qDataInit(vector3qData * vector, VectType type)
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
	vector->vector.pData = vector->vector3fData;

	// Set vector to 0
	vector->vector.pData[VECT_X] = 0;
	vector->vector.pData[VECT_Y] = 0;
	vector->vector.pData[VECT_Z] = 0;
}

void math_vectorUpdate(vector3fData * vector, float32_t i, float32_t j, float32_t k)
{
	vector->deltaTime = systemTime - vector->dataTime;
	vector->dataTime = systemTime;
	vector->vector.pData[VECT_X] = i;
	vector->vector.pData[VECT_Y] = j;
	vector->vector.pData[VECT_Z] = k;
}

void math_matrix3by3Init(matrix3by3 * matrix, MathYesNo identity)
{
	matrix->dataTime = systemTime;
	matrix->vector.numCols = 3;
	matrix->vector.numRows = 3;
	matrix->vector.pData = matrix->vector3fData;
	if(identity == MATH_NO)
	{
		matrix->vector.pData[Rxx] = 0;
		matrix->vector.pData[Ryx] = 0;
		matrix->vector.pData[Rzx] = 0;
		matrix->vector.pData[Rxy] = 0;
		matrix->vector.pData[Ryy] = 0;
		matrix->vector.pData[Rzy] = 0;
		matrix->vector.pData[Rxz] = 0;
		matrix->vector.pData[Ryz] = 0;
		matrix->vector.pData[Rzz] = 0;
	}
	else
	{
		matrix->vector.pData[Rxx] = 1;
		matrix->vector.pData[Ryx] = 0;
		matrix->vector.pData[Rzx] = 0;
		matrix->vector.pData[Rxy] = 0;
		matrix->vector.pData[Ryy] = 1;
		matrix->vector.pData[Rzy] = 0;
		matrix->vector.pData[Rxz] = 0;
		matrix->vector.pData[Ryz] = 0;
		matrix->vector.pData[Rzz] = 1;
	}
}


void math_generateRotationUpdateMatrix(float32_t x, float32_t y, float32_t z, matrix3by3 * matrix)
{
	matrix->dataTime = systemTime;
	matrix->vector3fData[0] = 1;
	matrix->vector3fData[1] = z;
	matrix->vector3fData[2] = -y;
	matrix->vector3fData[3] = -z;
	matrix->vector3fData[4] = 1;
	matrix->vector3fData[5] = x;
	matrix->vector3fData[6] = y;
	matrix->vector3fData[7] = -x;
	matrix->vector3fData[8] = 1;
}

// Multiply vector by scalar
arm_status math_multVectorScalar(vector3fData * vector, float32_t scalar)
{
	vector->vector.pData[VECT_X] = vector->vector.pData[VECT_X] * scalar;
	vector->vector.pData[VECT_Y] = vector->vector.pData[VECT_Y] * scalar;
	vector->vector.pData[VECT_Z] = vector->vector.pData[VECT_Z] * scalar;
	return SUCCESS;
}

// Calculate cross product of vectors A and B, store to vector C
arm_status math_vectCrossProduct(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC)
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
arm_status math_multMatrices(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC)
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

// Copy matrix A to matrix B
arm_status math_copyMatrix(matrix3by3 * matrixA, matrix3by3 * matrixB)
{
	matrixB->vector.pData[0] = matrixA->vector.pData[0];
	matrixB->vector.pData[1] = matrixA->vector.pData[1];
	matrixB->vector.pData[2] = matrixA->vector.pData[2];
	matrixB->vector.pData[3] = matrixA->vector.pData[3];
	matrixB->vector.pData[4] = matrixA->vector.pData[4];
	matrixB->vector.pData[5] = matrixA->vector.pData[5];
	matrixB->vector.pData[6] = matrixA->vector.pData[6];
	matrixB->vector.pData[7] = matrixA->vector.pData[7];
	matrixB->vector.pData[8] = matrixA->vector.pData[8];
	return ARM_MATH_SUCCESS;
}

// Multiply vectorA and matrix into vectorB
arm_status math_multVectorMatrix(vector3fData * vectorA, matrix3by3 * matrix, vector3fData * vectorB)
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
