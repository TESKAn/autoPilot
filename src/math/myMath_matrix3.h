/*
 * myMath_matrix3.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_MATRIX3_H_
#define MYMATH_MATRIX3_H_

typedef struct
{
	float32_t x;
	float32_t y;
	float32_t z;

}__attribute__((aligned(4),packed)) Vectorf1;

typedef struct
{
	Vectorf1 a;
	Vectorf1 b;
	Vectorf1 c;
}__attribute__((aligned(4),packed)) Matrixf;

Matrixf matrix3_init(int identity);
ErrorStatus matrix3_vectorMultiply(Matrixf * mat, Vectorf1 * vecIn, Vectorf1 * vecOut);
ErrorStatus matrix3_transposeVectorMultiply(Matrixf * mat, Vectorf1 * vecIn, Vectorf1 * vecOut);
ErrorStatus matrix3_MatrixMultiply(Matrixf * matA, Matrixf * matB, Matrixf * matC);
ErrorStatus matrix3_transpose(Matrixf * matA, Matrixf * matB);
ErrorStatus matrix3_copy(Matrixf * matA, Matrixf * matB);

#endif /* MYMATH_MATRIX3_H_ */
