/*
 * myMath_vec3.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_VEC3_H_
#define MYMATH_VEC3_H_


Vectorf vectorf_init(float32_t value);
Vectorui16 vectorui16_init(int16_t value);
ErrorStatus vectorf_dotProduct(Vectorf * vecA, Vectorf * vecB, float32_t * res);
ErrorStatus vectorf_crossProduct(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_scalarProduct(Vectorf * vecA, float32_t scalar, Vectorf * vecB);
ErrorStatus vectorf_substract(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_add(Vectorf * vecA, Vectorf * vecB, Vectorf * vecC);
ErrorStatus vectorf_normalize(Vectorf * vectorA);
float32_t vectorf_getNorm(Vectorf * vector);
ErrorStatus vectorf_copy(Vectorf * vecA, Vectorf * vecB);

#endif /* MYMATH_VEC3_H_ */
