/*
 * ahrs_math.c
 *
 *  Created on: Jan 1, 2013
 *      Author: Jure
 */

#include "allinclude.h"

arm_status ahrs_updateVectorPID(PI3Data* PID, vector3fData * errorVector, uint32_t deltaT)
{
	float32_t temp = 0;
	float32_t time = (float32_t)deltaT / 100;
	// Limit error
	errorVector->vector.pData[VECT_X] = ahrs_limitFloat(errorVector->vector.pData[VECT_X], PID->eMax, PID->eMin);
	errorVector->vector.pData[VECT_Y] = ahrs_limitFloat(errorVector->vector.pData[VECT_Y], PID->eMax, PID->eMin);
	errorVector->vector.pData[VECT_Z] = ahrs_limitFloat(errorVector->vector.pData[VECT_Z], PID->eMax, PID->eMin);

	// Update PID x
	// Calculate error I add
	temp = errorVector->vector.pData[VECT_X] * PID->Kix * time * SYSTIME_TOSECONDS;
	// Add
	PID->Ix = PID->Ix + temp;
	// Limit I part
	PID->Ix = ahrs_limitFloat(PID->Ix, PID->maxIx, PID->minIx);
	// Calculate error P add
	temp = errorVector->vector.pData[VECT_X] * PID->Kpx;
	PID->Px = temp;
	// Add to PI result
	PID->Rx = PID->Px + PID->Ix;
	// Limit
	ahrs_limitFloat(PID->Rx, PID->rMax, PID->rMin);

	// Update PID y
	// Calculate error I add
	temp = errorVector->vector.pData[VECT_Y] * PID->Kiy * time * SYSTIME_TOSECONDS;
	// Add
	PID->Iy = PID->Iy + temp;
	// Limit I part
	PID->Iy = ahrs_limitFloat(PID->Iy, PID->maxIy, PID->minIy);
	// Calculate error P add
	temp = errorVector->vector.pData[VECT_Y] * PID->Kpy;
	PID->Py = temp;
	// Add to PI result
	PID->Ry = PID->Py + PID->Iy;
	// Limit
	ahrs_limitFloat(PID->Ry, PID->rMax, PID->rMin);

	// Update PID z
	// Calculate error I add
	temp = errorVector->vector.pData[VECT_Z] * PID->Kiz * time * SYSTIME_TOSECONDS;
	// Add
	PID->Iz = PID->Iz + temp;
	// Limit I part
	PID->Iz = ahrs_limitFloat(PID->Iz, PID->maxIz, PID->minIz);
	// Calculate error P add
	temp = errorVector->vector.pData[VECT_Z] * PID->Kpz;
	PID->Pz = temp;
	// Add to PI result
	PID->Rz = PID->Pz + PID->Iz;
	// Limit
	ahrs_limitFloat(PID->Rz, PID->rMax, PID->rMin);

	return ARM_MATH_SUCCESS;
}

float32_t ahrs_limitFloat(float32_t number, float32_t max, float32_t min)
{
	if(number > max) return max;
	else if(number < min) return min;
	else return number;
}

void updateScaledVector(vector3fData * vector, uint16_t x, uint16_t y, uint16_t z, float rate)
{
	vector->vector.pData[VECT_X] = (float) ((int16_t)x) * rate;
	vector->vector.pData[VECT_Y] = (float) ((int16_t)y) * rate;
	vector->vector.pData[VECT_Z] = (float) ((int16_t)z) * rate;
	// Store delta time
	vector->deltaTime = sensorAcquisitionTime - vector->dataTime;
	// Store time
	vector->dataTime = sensorAcquisitionTime;
}


void ahrs_vectorDataInit(vector3fData * vector, VectType type)
{
	vector->dataTime = getSystemTime();
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
	vector->vector.pData[0] = 0;
	vector->vector.pData[1] = 0;
	vector->vector.pData[2] = 0;
}

void ahrs_vector3qDataInit(vector3qData * vector, VectType type)
{
	vector->dataTime = getSystemTime();
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
	vector->vector.pData[0] = 0;
	vector->vector.pData[1] = 0;
	vector->vector.pData[2] = 0;
}

void ahrs_vectorUpdate(vector3fData * vector, float32_t i, float32_t j, float32_t k)
{
	uint32_t timeCalculation = vector->dataTime;
	vector->dataTime = getSystemTime();
	vector->deltaTime = vector->dataTime - timeCalculation;
	vector->vector.pData[0] = i;
	vector->vector.pData[1] = j;
	vector->vector.pData[2] = k;
}

void ahrs_matrix3by3_init(matrix3by3 * matrix)
{
	uint32_t timeCalculation = matrix->dataTime;
	matrix->dataTime = getSystemTime();
	matrix->deltaTime = matrix->dataTime - timeCalculation;
	matrix->vector.numCols = 3;
	matrix->vector.numRows = 3;
	matrix->vector.pData = matrix->vector3fData;
}

void ahrs_generate_rotationMatrix(matrix3by3 * matrix, float roll, float pitch, float yaw)
{
	uint32_t timeCalculation = matrix->dataTime;
	matrix->dataTime = getSystemTime();
	matrix->deltaTime = matrix->dataTime - timeCalculation;

	float cp = arm_cos_f32(pitch);
	float sp = arm_sin_f32(pitch);
	float cr = arm_cos_f32(roll);
	float sr = arm_sin_f32(roll);
	float cy = arm_cos_f32(yaw);
	float sy = arm_sin_f32(yaw);

	matrix->vector3fData[0] = cp * cy;
	matrix->vector3fData[1] = cp * sy;
	matrix->vector3fData[2] = -sp;
	matrix->vector3fData[3] = (sr * sp * cy) - (cr * sy);
	matrix->vector3fData[4] = (sr * sp * sy) + (cr * cy);
	matrix->vector3fData[5] = sr * cp;
	matrix->vector3fData[6] = (cr * sp * cy) + (sr * sy);;
	matrix->vector3fData[7] = (cr * sp * sy) - (sr * cy);
	matrix->vector3fData[8] = cr * cp;
}

void ahrs_generate_rotationUpdateMatrix(float32_t x, float32_t y, float32_t z, matrix3by3 * matrix)
{
	matrix->dataTime = getSystemTime();
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

// Substract two vectors, A - B
arm_status ahrs_vector_substract(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC)
{
	vectorC->vector.pData[VECT_X] = vectorA->vector.pData[VECT_X]  - vectorB->vector.pData[VECT_X];
	vectorC->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Y]  - vectorB->vector.pData[VECT_Y];
	vectorC->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_Z]  - vectorB->vector.pData[VECT_Z];
	vectorC->dataTime = getSystemTime();
	return ARM_MATH_SUCCESS;
}

// Multiply vector by scalar
arm_status ahrs_mult_vector_scalar(vector3fData * vector, float32_t scalar)
{
	vector->vector.pData[VECT_X] = vector->vector.pData[VECT_X] * scalar;
	vector->vector.pData[VECT_Y] = vector->vector.pData[VECT_Y] * scalar;
	vector->vector.pData[VECT_Z] = vector->vector.pData[VECT_Z] * scalar;
	return ARM_MATH_SUCCESS;
}

// Calculate dot product of vectors A and B, store to vector C
arm_status ahrs_vect_dot_product(vector3fData * vectorA, vector3fData * vectorB, float32_t * scalarC)
{
	float32_t temp = 0;

	*scalarC = vectorA->vector.pData[VECT_X]*vectorB->vector.pData[VECT_X];
	temp = vectorA->vector.pData[VECT_Y]*vectorB->vector.pData[VECT_Y];
	*scalarC = *scalarC + temp;
	temp = vectorA->vector.pData[VECT_Z]*vectorB->vector.pData[VECT_Z];
	*scalarC = *scalarC + temp;
	return ARM_MATH_SUCCESS;
}

// Calculate cross product of vectors A and B, store to vector C
arm_status ahrs_vect_cross_product(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC)
{
	// Result X = Ay*Bz - Az*By
	vectorC->vector.pData[VECT_X] = vectorA->vector.pData[VECT_Y]*vectorB->vector.pData[VECT_Z] - vectorA->vector.pData[VECT_Z]*vectorB->vector.pData[VECT_Y];
	// Result Y = Az*Bx - Ax*Bz
	vectorC->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Z]*vectorB->vector.pData[VECT_X] - vectorA->vector.pData[VECT_X]*vectorB->vector.pData[VECT_Z];
	// Result Z = Ax*By - Ay*Bx
	vectorC->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_X]*vectorB->vector.pData[VECT_Y] - vectorA->vector.pData[VECT_Y]*vectorB->vector.pData[VECT_X];

	vectorC->dataTime = getSystemTime();
	return ARM_MATH_SUCCESS;
}

// Multiply matrices A and B into matrix C
arm_status ahrs_mult_matrixes(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC)
{/*
	// Multiply matrices
	matrixC->vector.pData[Rxx] = matrixA->vector.pData[Rxx] * matrixB->vector.pData[Rxx] +
			matrixA->vector.pData[Rxy] * matrixB->vector.pData[Ryx] +
			matrixA->vector.pData[Rxz] * matrixB->vector.pData[Rzx] ;

	matrixC->vector.pData[Ryx] = matrixA->vector.pData[Ryx] * matrixB->vector.pData[Rxx] +
			matrixA->vector.pData[Ryy] * matrixB->vector.pData[Ryx] +
			matrixA->vector.pData[Ryz] * matrixB->vector.pData[Rzx] ;

	matrixC->vector.pData[Rzx] = matrixA->vector.pData[Rzx] * matrixB->vector.pData[Rxx] +
			matrixA->vector.pData[Rzy] * matrixB->vector.pData[Ryx] +
			matrixA->vector.pData[Rzz] * matrixB->vector.pData[Rzx] ;

	matrixC->vector.pData[Rxy] = matrixA->vector.pData[Rxx] * matrixB->vector.pData[Rxy] +
			matrixA->vector.pData[Rxy] * matrixB->vector.pData[Ryy] +
			matrixA->vector.pData[Rxz] * matrixB->vector.pData[Rzy] ;

	matrixC->vector.pData[Ryy] = matrixA->vector.pData[Ryx] * matrixB->vector.pData[Rxy] +
			matrixA->vector.pData[Ryy] * matrixB->vector.pData[Ryy] +
			matrixA->vector.pData[Ryz] * matrixB->vector.pData[Rzy] ;

	matrixC->vector.pData[Rzy] = matrixA->vector.pData[Rzx] * matrixB->vector.pData[Rxy] +
			matrixA->vector.pData[Rzy] * matrixB->vector.pData[Ryy] +
			matrixA->vector.pData[Rzz] * matrixB->vector.pData[Rzy] ;

	matrixC->vector.pData[Rxz] = matrixA->vector.pData[Rxx] * matrixB->vector.pData[Rxz] +
			matrixA->vector.pData[Rxy] * matrixB->vector.pData[Ryz] +
			matrixA->vector.pData[Rxz] * matrixB->vector.pData[Rzz] ;

	matrixC->vector.pData[Ryz] = matrixA->vector.pData[Ryx] * matrixB->vector.pData[Rxz] +
			matrixA->vector.pData[Ryy] * matrixB->vector.pData[Ryz] +
			matrixA->vector.pData[Ryz] * matrixB->vector.pData[Rzz] ;

	matrixC->vector.pData[Rzz] = matrixA->vector.pData[Rzx] * matrixB->vector.pData[Rxz] +
			matrixA->vector.pData[Rzy] * matrixB->vector.pData[Ryz] +
			matrixA->vector.pData[Rzz] * matrixB->vector.pData[Rzz] ;
*/

	arm_status status = arm_mat_mult_f32(&(matrixA->vector), &(matrixB->vector), &(matrixC->vector));
	if(status == ARM_MATH_SUCCESS)
	{
		// Update time when we multiplied
		matrixC->dataTime = getSystemTime();
	}
	return ARM_MATH_SUCCESS;
}

// Multiply vectorA and matrix into vectorB
arm_status ahrs_mult_vector_matrix(matrix3by3 * matrixA, vector3fData * vectorA, vector3fData * vectorB)
{
	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;
	// Multiply
	a = matrixA->vector.pData[Rxx] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Rxy] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Rxz] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_X] = a + b + c;

	a = matrixA->vector.pData[Ryx] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Ryy] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Ryz] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_Y] = a + b + c;

	a = matrixA->vector.pData[Rzx] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Rzy] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Rzz] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_Z] = a + b + c;

	vectorB->dataTime = vectorA->dataTime;

	//status = arm_mat_mult_f32(&(vectorA->vector), &(matrixA->vector), &(vectorB->vector));
	//status = arm_mat_mult_f32(&(matrixA->vector), &(vectorA->vector), &(vectorB->vector));
	return ARM_MATH_SUCCESS;
}

// Multiply vectorA and matrix transpose into vectorB
arm_status ahrs_mult_vector_matrix_transpose(matrix3by3 * matrixA, vector3fData * vectorA, vector3fData * vectorB)
{
	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;
	a = matrixA->vector.pData[Rxx] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Ryx] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Rzx] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_X] = a + b + c;

	a = matrixA->vector.pData[Rxy] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Ryy] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Rzy] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_Y] = a + b + c;

	a = matrixA->vector.pData[Rxz] * vectorA->vector.pData[VECT_X];
	b = matrixA->vector.pData[Ryz] * vectorA->vector.pData[VECT_Y];
	c = matrixA->vector.pData[Rzz] * vectorA->vector.pData[VECT_Z];
	vectorB->vector.pData[VECT_Z] = a + b + c;

	vectorB->dataTime = vectorA->dataTime;

	//status = arm_mat_mult_f32(&(vectorA->vector), &(matrixA->vector), &(vectorB->vector));
	//status = arm_mat_mult_f32(&(matrixA->vector), &(vectorA->vector), &(vectorB->vector));
	return ARM_MATH_SUCCESS;
}

arm_status ahrs_matrix_transponse(matrix3by3 * matrixA, matrix3by3 * matrixB)
{
	arm_status status;

	status = arm_mat_trans_f32(&(matrixA->vector), &(matrixB->vector));

	return status;
}

// Normalize vector A to 1
arm_status ahrs_normalize_vector_taylor(vector3fData * vectorA)
{
	arm_status status = ARM_MATH_SUCCESS;
	float32_t scale = 0;
	scale = vectorA->vector.pData[VECT_X];
	scale = scale * vectorA->vector.pData[VECT_X];
	scale = scale +(vectorA->vector.pData[VECT_Y] * vectorA->vector.pData[VECT_Y]);
	scale = scale + +(vectorA->vector.pData[VECT_Z] * vectorA->vector.pData[VECT_Z]);
	// Calculate 3 - scalar
	scale = 3 - scale;
	// Calculate one half
	scale = scale / 2;

	vectorA->vector.pData[VECT_X] = vectorA->vector.pData[VECT_X] * scale;
	vectorA->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Y] * scale;
	vectorA->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_Z] * scale;

	return status;
}

// Normalize vector A to 1
arm_status ahrs_normalize_vector(vector3fData * vectorA)
{
	arm_status status = ARM_MATH_SUCCESS;
	float32_t scale = 0;
	float32_t xx = 0;
	float32_t yy = 0;
	float32_t zz = 0;
	xx = vectorA->vector.pData[VECT_X] * vectorA->vector.pData[VECT_X];
	yy = vectorA->vector.pData[VECT_Y] * vectorA->vector.pData[VECT_Y];
	zz = vectorA->vector.pData[VECT_Z] * vectorA->vector.pData[VECT_Z];
	scale = xx + yy + zz;
	if(scale > 0)
	{
		//scale = 1/sqrtf(scale);
		scale = ahrs_fast_inverse_root(scale);
	}
	else
	{
		scale = 0;
		status = ARM_MATH_SINGULAR;
	}
	vectorA->vector.pData[VECT_X] = vectorA->vector.pData[VECT_X] * scale;
	vectorA->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Y] * scale;
	vectorA->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_Z] * scale;

	return status;
}

float32_t ahrs_get_vector_norm(vector3fData * vector)
{
	float32_t result = 0;
	result = vector->vector.pData[VECT_X] * vector->vector.pData[VECT_X];
	result = result + vector->vector.pData[VECT_Y] * vector->vector.pData[VECT_Y];
	result = result + vector->vector.pData[VECT_Z] * vector->vector.pData[VECT_Z];
	result = sqrtf(result);
	return result;
}

// Copy vector A to B
arm_status ahrs_copy_vector(vector3fData * vectorA, vector3fData * vectorB)
{
	vectorB->vector.pData[VECT_X] = vectorA->vector.pData[VECT_X];
	vectorB->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Y];
	vectorB->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_Z];
	vectorB->dataTime = vectorA->dataTime;
	return ARM_MATH_SUCCESS;
}

// Calculate vector magnitude
float32_t ahrs_vector_magnitude(vector3fData * vectorA)
{
	float32_t result = 0;
	result = vectorA->vector.pData[VECT_X] * vectorA->vector.pData[VECT_X];
	result = result + vectorA->vector.pData[VECT_Y] * vectorA->vector.pData[VECT_Y];
	result = result + vectorA->vector.pData[VECT_Z] * vectorA->vector.pData[VECT_Z];
	result = sqrtf(result);
	return result;
}

// Calculate fast inverse square root of a number
float32_t ahrs_fast_inverse_root(float32_t num)
{
	float32_t result = 0;
	union { float32_t f; uint32_t u; } y = {num};
	y.u = (uint32_t)(0x5F1FFF77) - (y.u >> 1);
	result = 0.703974056f * y.f * (2.38919526f - (num * y.f * y.f));
	return result;
}

// Add two vectors, A + B
arm_status ahrs_vector_add(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC)
{
	vectorC->vector.pData[VECT_X] = vectorA->vector.pData[VECT_X]  + vectorB->vector.pData[VECT_X];
	vectorC->vector.pData[VECT_Y] = vectorA->vector.pData[VECT_Y]  + vectorB->vector.pData[VECT_Y];
	vectorC->vector.pData[VECT_Z] = vectorA->vector.pData[VECT_Z]  + vectorB->vector.pData[VECT_Z];
	vectorC->dataTime = getSystemTime();
	return ARM_MATH_SUCCESS;
}
