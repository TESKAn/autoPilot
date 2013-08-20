/*
 * myMath_vec3.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_VEC3_H_
#define MYMATH_VEC3_H_

typedef struct
{
	float32_t x;
	float32_t y;
	float32_t z;

}__attribute__((aligned(4),packed)) Vectorf;

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;

}__attribute__((aligned(4),packed)) Vectorui16;

Vectorf vectorf_init();
Vectorui16 vectorui16_init();
ErrorStatus vectorf_dotProduct(Vectorf * vecA, Vectorf * vecB, float32_t * res);
ErrorStatus vectorf_crossProduct(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_scalarProduct(Vectorf * vecA, float32_t scalar, Vectorf * vecB);
ErrorStatus vectorf_substract(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_add(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_normalize(Vectorf * vectorA);
float32_t vectorf_getNorm(Vectorf * vector);
ErrorStatus vectorf_copy(Vectorf * vecA, Vectorf * vecB);

#endif /* MYMATH_VEC3_H_ */
