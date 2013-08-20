/*
 * myMath_matrix3.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "customTypedefs.h"
#include "myMath.h"
#include "myMath_vec3.h"
#include "myMath_matrix3.h"

// Create and initialize new 3x3 matrix
// if identity == 0, init matrix with all zeroes
// if identity == 1, init identity matrix
Matrixf matrix3_init(int identity)
{
	Matrixf mat;
	mat.a.x = 0;
	mat.a.y = 0;
	mat.a.z = 0;

	mat.b.x = 0;
	mat.b.y = 0;
	mat.b.z = 0;

	mat.c.x = 0;
	mat.c.y = 0;
	mat.c.z = 0;

	if(0 != identity)
	{
		mat.a.x = 1;
		mat.b.y = 1;
		mat.c.z = 1;
	}

	return mat;
}

ErrorStatus matrix3_vectorMultiply(Matrixf * mat, Vectorf1 * vecIn, Vectorf1 * vecOut)
{
	ErrorStatus status = ERROR;
	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;

	// Calculate vector by multiplying given vector with given matrix
	a = mat->a.x * vecIn->x;
	b = mat->a.y * vecIn->y;
	c = mat->a.z * vecIn->z;
	vecOut->x = a + b + c;

	a = mat->b.x * vecIn->x;
	b = mat->b.y * vecIn->y;
	c = mat->b.z * vecIn->z;
	vecOut->y = a + b + c;

	a = mat->c.x * vecIn->x;
	b = mat->c.y * vecIn->y;
	c = mat->c.z * vecIn->z;

	vecOut->z = a + b + c;

	status = SUCCESS;

	return status;
}

ErrorStatus matrix3_transposeVectorMultiply(Matrixf * mat, Vectorf1 * vecIn, Vectorf1 * vecOut)
{
	ErrorStatus status = ERROR;

	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;

	// Calculate vector by multiplying given vector with given matrix transpose
	a = mat->a.x * vecIn->x;
	b = mat->b.x * vecIn->y;
	c = mat->c.x * vecIn->z;
	vecOut->x = a + b + c;
	a = mat->a.y * vecIn->x;
	b = mat->b.y * vecIn->y;
	c = mat->c.y * vecIn->z;
	vecOut->y = a + b + c;
	a = mat->a.z * vecIn->x;
	b = mat->b.z * vecIn->y;
	c = mat->c.z * vecIn->z;
	vecOut->z = a + b + c;

	status = SUCCESS;

	return status;
}

// Multiply matrices A and B into matrix C
ErrorStatus matrix3_MatrixMultiply(Matrixf * matA, Matrixf * matB, Matrixf * matC)
{
	ErrorStatus status = ERROR;

	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;

	// Calculate resulting matrix by multiplying matrices A and B
	// Row A.a, col B.x
	a = matA->a.x * matB->a.x;
	b = matA->a.y * matB->b.x;
	c = matA->a.z * matB->c.x;
	matC->a.x = a + b + c;
	a = matA->a.x * matB->a.y;
	b = matA->a.y * matB->b.y;
	c = matA->a.z * matB->c.y;
	matC->a.y = a + b + c;
	a = matA->a.x * matB->a.z;
	b = matA->a.y * matB->b.z;
	c = matA->a.z * matB->c.z;
	matC->a.z = a + b + c;

	a = matA->b.x * matB->a.x;
	b = matA->b.y * matB->b.x;
	c = matA->b.z * matB->c.x;
	matC->b.x = a + b + c;
	a = matA->b.x * matB->a.y;
	b = matA->b.y * matB->b.y;
	c = matA->b.z * matB->c.y;
	matC->b.y = a + b + c;
	a = matA->b.x * matB->a.z;
	b = matA->b.y * matB->b.z;
	c = matA->b.z * matB->c.z;
	matC->b.z = a + b + c;

	a = matA->c.x * matB->a.x;
	b = matA->c.y * matB->b.x;
	c = matA->c.z * matB->c.x;
	matC->c.x = a + b + c;
	a = matA->c.x * matB->a.y;
	b = matA->c.y * matB->b.y;
	c = matA->c.z * matB->c.y;
	matC->c.y = a + b + c;
	a = matA->c.x * matB->a.z;
	b = matA->c.y * matB->b.z;
	c = matA->c.z * matB->c.z;
	matC->c.z = a + b + c;

	status = SUCCESS;

	return status;
}

ErrorStatus matrix3_transpose(Matrixf * matA, Matrixf * matB)
{
	ErrorStatus status = ERROR;

	matA->a.x = matB->a.x;
	matA->a.y = matB->b.x;
	matA->a.z = matB->c.x;

	matA->b.x = matB->a.y;
	matA->b.y = matB->b.y;
	matA->b.z = matB->c.y;

	matA->c.x = matB->a.z;
	matA->c.y = matB->b.z;
	matA->c.z = matB->c.z;

	status = SUCCESS;

	return status;
}

ErrorStatus matrix3_copy(Matrixf * matA, Matrixf * matB)
{
	ErrorStatus status = ERROR;

	matA->a.x = matB->a.x;
	matA->a.y = matB->a.y;
	matA->a.z = matB->a.z;

	matA->b.x = matB->b.x;
	matA->b.y = matB->b.y;
	matA->b.z = matB->b.z;

	matA->c.x = matB->c.x;
	matA->c.y = matB->c.y;
	matA->c.z = matB->c.z;

	status = SUCCESS;

	return status;
}

