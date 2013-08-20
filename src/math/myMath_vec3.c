/*
 * myMath_vec3.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "customTypedefs.h"
#include "myMath.h"
#include "myMath_vec3.h"

Vectorf vectorf_init()
{
	Vectorf vec;
	vec.x = 0;
	vec.y = 0;
	vec.z = 0;
	return vec;
}

Vectorui16 vectorui16_init()
{
	Vectorui16 vec;
	vec.x = 0;
	vec.y = 0;
	vec.z = 0;
	return vec;
}

ErrorStatus vectorf_dotProduct(Vectorf * vecA, Vectorf * vecB, float32_t * res)
{
	ErrorStatus status = ERROR;

	*res = vecA->x * vecB->x + vecA->y * vecB->y +  vecA->z * vecB->z;

	status = SUCCESS;

	return status;
}

ErrorStatus vectorf_crossProduct(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC)
{
	ErrorStatus status = ERROR;

	// Result X = Ay*Bz - Az*By
	vecC->x = vecA->y * vecB->z - vecA->z * vecB->y;
	// Result Y = Az*Bx - Ax*Bz
	vecC->y = vecA->z * vecB->x - vecA->x * vecB->z;
	// Result Z = Ax*By - Ay*Bx
	vecC->z = vecA->x * vecB->y - vecA->y * vecB->x;

	status = SUCCESS;

	return status;
}

ErrorStatus vectorf_scalarProduct(Vectorf * vecA, float32_t scalar, Vectorf * vecB)
{
	ErrorStatus status = ERROR;

	vecB->x = vecA->x * scalar;
	vecB->y = vecA->y * scalar;
	vecB->z = vecA->z * scalar;

	status = SUCCESS;

	return status;
}

ErrorStatus vectorf_substract(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC)
{
	ErrorStatus status = ERROR;
	vecC->x = vecA->x - vecB->x;
	vecC->y = vecA->y - vecB->y;
	vecC->z = vecA->z - vecB->z;

	status = SUCCESS;

	return status;
}

ErrorStatus vectorf_add(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC)
{
	ErrorStatus status = ERROR;
	vecC->x = vecA->x + vecB->x;
	vecC->y = vecA->y + vecB->y;
	vecC->z = vecA->z + vecB->z;

	status = SUCCESS;

	return status;
}

ErrorStatus vectorf_normalize(Vectorf * vectorA)
{
	ErrorStatus status = ERROR;
	float32_t scale = 0;
	float32_t xx = 0;
	float32_t yy = 0;
	float32_t zz = 0;
	xx = vectorA->x * vectorA->x;
	yy = vectorA->y * vectorA->y;
	zz = vectorA->z * vectorA->z;
	scale = xx + yy + zz;
	if(scale > 0)
	{
		scale = math_fastInverseRoot(scale);
		status = SUCCESS;
	}
	else
	{
		scale = 0;
		status = ERROR;
	}
	vectorA->x = vectorA->x * scale;
	vectorA->y = vectorA->y * scale;
	vectorA->z = vectorA->z * scale;

	return status;
}

float32_t vectorf_getNorm(Vectorf * vector)
{
	float32_t result = 0;
	result = vector->x * vector->x;
	result = result + vector->y * vector->y;
	result = result + vector->z * vector->z;
	result = sqrtf(result);
	return result;
}

ErrorStatus vectorf_copy(Vectorf * vecA, Vectorf * vecB)
{
	ErrorStatus status = ERROR;
	vecA->x = vecB->x;
	vecA->y = vecB->y;
	vecA->z = vecB->z;

	status = SUCCESS;

	return status;
}
