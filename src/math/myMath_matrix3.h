/*
 * myMath_matrix3.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_MATRIX3_H_
#define MYMATH_MATRIX3_H_

Matrixf matrix3_init(int identity);
ErrorStatus matrix3_vectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut);
ErrorStatus matrix3_transposeVectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut);
ErrorStatus matrix3_MatrixMultiply(Matrixf * matA, Matrixf * matB, Matrixf * matC);
ErrorStatus matrix3_transpose(Matrixf * matA, Matrixf * matB);
ErrorStatus matrix3_copy(Matrixf * matA, Matrixf * matB);

#endif /* MYMATH_MATRIX3_H_ */
