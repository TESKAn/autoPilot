/*
 * myMath_matrix3.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "allinclude.h"
#include "myMath_typedefs.h"
#include "myMath.h"
#include "myMath_vec3.h"
#include "myMath_matrix3.h"

// Create and initialize new 3x3 matrix
// if identity == 0, init matrix with all zeroes
// if identity == 1, init identity matrix
ErrorStatus matrix3_init(int identity, Matrixf * mat)
{
	mat->a.x = 0;
	mat->a.y = 0;
	mat->a.z = 0;

	mat->b.x = 0;
	mat->b.y = 0;
	mat->b.z = 0;

	mat->c.x = 0;
	mat->c.y = 0;
	mat->c.z = 0;

	if(0 != identity)
	{
		mat->a.x = 1;
		mat->b.y = 1;
		mat->c.z = 1;
	}
	return SUCCESS;
}

ErrorStatus matrix3_sumAB(Matrixf * matA, Matrixf * matB, Matrixf * matC)
{
	ErrorStatus status = ERROR;
	matC->a.x = matA->a.x + matB->a.x;
	matC->a.y = matA->a.y + matB->a.y;
	matC->a.z = matA->a.z + matB->a.z;

	matC->b.x = matA->b.x + matB->b.x;
	matC->b.y = matA->b.y + matB->b.y;
	matC->b.z = matA->b.z + matB->b.z;

	matC->c.x = matA->c.x + matB->c.x;
	matC->c.y = matA->c.y + matB->c.y;
	matC->c.z = matA->c.z + matB->c.z;

	return status;
}

ErrorStatus matrix3_vectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut)
{
	ErrorStatus status = ERROR;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;
	// Use temporary vector to store data
	Vectorf result;

	// Calculate vector by multiplying given vector with given matrix
	a = mat->a.x * vecIn->x;
	b = mat->a.y * vecIn->y;
	c = mat->a.z * vecIn->z;
	result.x = a + b + c;

	a = mat->b.x * vecIn->x;
	b = mat->b.y * vecIn->y;
	c = mat->b.z * vecIn->z;
	result.y = a + b + c;

	a = mat->c.x * vecIn->x;
	b = mat->c.y * vecIn->y;
	c = mat->c.z * vecIn->z;

	result.z = a + b + c;

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
		vectorf_copy(&result, vecOut);
	}

	return status;
}

ErrorStatus matrix3_transposeVectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut)
{
	ErrorStatus status = ERROR;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;
	// Use temporary vector to store data
	Vectorf result;

	// Calculate vector by multiplying given vector with given matrix transpose
	a = mat->a.x * vecIn->x;
	b = mat->b.x * vecIn->y;
	c = mat->c.x * vecIn->z;
	result.x = a + b + c;
	a = mat->a.y * vecIn->x;
	b = mat->b.y * vecIn->y;
	c = mat->c.y * vecIn->z;
	result.y = a + b + c;
	a = mat->a.z * vecIn->x;
	b = mat->b.z * vecIn->y;
	c = mat->c.z * vecIn->z;
	result.z = a + b + c;

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
		vectorf_copy(&result, vecOut);
	}

	return status;
}

// Multiply matrix A with scalar S into matrix B
ErrorStatus matrix3_scalarMultiply(Matrixf * A, float32_t S, Matrixf * B)
{
	ErrorStatus status = ERROR;
	B->a.x = A->a.x * S;
	B->a.y = A->a.y * S;
	B->a.z = A->a.z * S;
	B->b.x = A->b.x * S;
	B->b.y = A->b.y * S;
	B->b.z = A->b.z * S;
	B->c.x = A->c.x * S;
	B->c.y = A->c.y * S;
	B->c.z = A->c.z * S;

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
	}

	return status;
}

// Multiply matrices A and B into matrix C
ErrorStatus matrix3_MatrixMultiply(Matrixf * matA, Matrixf * matB, Matrixf * matC)
{
	ErrorStatus status = ERROR;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;

	Matrixf matX;

	// Calculate resulting matrix by multiplying matrices A and B
	// Row A.a, col B.x
	a = matA->a.x * matB->a.x;
	b = matA->a.y * matB->b.x;
	c = matA->a.z * matB->c.x;
	matX.a.x = a + b + c;
	a = matA->a.x * matB->a.y;
	b = matA->a.y * matB->b.y;
	c = matA->a.z * matB->c.y;
	matX.a.y = a + b + c;
	a = matA->a.x * matB->a.z;
	b = matA->a.y * matB->b.z;
	c = matA->a.z * matB->c.z;
	matX.a.z = a + b + c;

	a = matA->b.x * matB->a.x;
	b = matA->b.y * matB->b.x;
	c = matA->b.z * matB->c.x;
	matX.b.x = a + b + c;
	a = matA->b.x * matB->a.y;
	b = matA->b.y * matB->b.y;
	c = matA->b.z * matB->c.y;
	matX.b.y = a + b + c;
	a = matA->b.x * matB->a.z;
	b = matA->b.y * matB->b.z;
	c = matA->b.z * matB->c.z;
	matX.b.z = a + b + c;

	a = matA->c.x * matB->a.x;
	b = matA->c.y * matB->b.x;
	c = matA->c.z * matB->c.x;
	matX.c.x = a + b + c;
	a = matA->c.x * matB->a.y;
	b = matA->c.y * matB->b.y;
	c = matA->c.z * matB->c.y;
	matX.c.y = a + b + c;
	a = matA->c.x * matB->a.z;
	b = matA->c.y * matB->b.z;
	c = matA->c.z * matB->c.z;
	matX.c.z = a + b + c;

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
		matC->a.x = matX.a.x;
		matC->a.y = matX.a.y;
		matC->a.z = matX.a.z;

		matC->b.x = matX.b.x;
		matC->b.y = matX.b.y;
		matC->b.z = matX.b.z;

		matC->c.x = matX.c.x;
		matC->c.y = matX.c.y;
		matC->c.z = matX.c.z;
	}

	return status;
}

ErrorStatus matrix3_transpose(Matrixf * matA, Matrixf * matB)
{
	Matrixf matX;

	matX.a.x = matB->a.x;
	matX.a.y = matB->b.x;
	matX.a.z = matB->c.x;

	matX.b.x = matB->a.y;
	matX.b.y = matB->b.y;
	matX.b.z = matB->c.y;

	matX.c.x = matB->a.z;
	matX.c.y = matB->b.z;
	matX.c.z = matB->c.z;

	matB->a.x = matX.a.x;
	matB->a.y = matX.a.y;
	matB->a.z = matX.a.z;

	matB->b.x = matX.b.x;
	matB->b.y = matX.b.y;
	matB->b.z = matX.b.z;

	matB->c.x = matX.c.x;
	matB->c.y = matX.c.y;
	matB->c.z = matX.c.z;

	return SUCCESS;
}

// Copy matrix A into matrix B
ErrorStatus matrix3_copy(Matrixf * matA, Matrixf * matB)
{
	matB->a.x = matA->a.x;
	matB->a.y = matA->a.y;
	matB->a.z = matA->a.z;

	matB->b.x = matA->b.x;
	matB->b.y = matA->b.y;
	matB->b.z = matA->b.z;

	matB->c.x = matA->c.x;
	matB->c.y = matA->c.y;
	matB->c.z = matA->c.z;

	return SUCCESS;
}

// Make matrix orthogonal and normalized
ErrorStatus matrix3_normalizeOrthogonalizeMatrix(Matrixf * rotMatrix, float32_t maxError)
{
	ErrorStatus status = SUCCESS;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	float32_t error = 0;
	Vectorf tempVector = vectorf_init(0);
	Vectorf tempVector1 = vectorf_init(0);

	// Calculate error = X.Y
	status = vectorf_dotProduct(&rotMatrix->a, &rotMatrix->b, &error);
	// If error is larger than we permit, return ERROR
	/*
	if(maxError < error)
	{
		return ERROR;
	}*/
	// Add half error to X, half to Y
	error = error / 2;
	status = vectorf_scalarProduct(&rotMatrix->a, error, &tempVector);
	status = vectorf_scalarProduct(&rotMatrix->b, error, &tempVector1);

	status = vectorf_substract(&rotMatrix->a, &tempVector1, &rotMatrix->a);
	status = vectorf_substract(&rotMatrix->b, &tempVector, &rotMatrix->b);
	// Normalize a, b
	status = vectorf_normalize(&rotMatrix->a);
	status = vectorf_normalize(&rotMatrix->b);
	// Calculate Z as cross product of X and Y
	status = vectorf_crossProduct(&rotMatrix->a, &rotMatrix->b, &rotMatrix->c);
	// Normalize c
	status = vectorf_normalize(&rotMatrix->c);

	/*
	status = vectorf_dotProduct(&rotMatrix->a, &rotMatrix->a, &error);
	error = 3 - error;
	error = error / 2;
	// If error is larger than we permit, return ERROR

	if(maxError < error)
	{
		return ERROR;
	}*//*
	status = vectorf_scalarProduct(&rotMatrix->a, error, &rotMatrix->a);
	// Normalize b
	status = vectorf_dotProduct(&rotMatrix->b, &rotMatrix->b, &error);
	error = 3 - error;
	error = error / 2;
	// If error is larger than we permit, return ERROR

	if(maxError < error)
	{
		return ERROR;
	}
	status = vectorf_scalarProduct(&rotMatrix->b, error, &rotMatrix->b);
	// Calculate Z as cross product of X and Y
	status = vectorf_crossProduct(&rotMatrix->a, &rotMatrix->b, &rotMatrix->c);
	// Normalize c
	status = vectorf_dotProduct(&rotMatrix->c, &rotMatrix->c, &error);
	error = 3 - error;
	error = error / 2;
	// If error is larger than we permit, return ERROR

	if(maxError < error)
	{
		return ERROR;
	}*//*
	status = vectorf_scalarProduct(&rotMatrix->c, error, &rotMatrix->c);
*/
	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
	}

	return status;
}
