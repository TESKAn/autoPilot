/*
 * myMath_matrix3.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_MATRIX3_H_
#define MYMATH_MATRIX3_H_

ErrorStatus matrix3_init(int identity, Matrixf * matrix);
ErrorStatus matrix3_sumAB(Matrixf * matA, Matrixf * matB, Matrixf * matC);
ErrorStatus matrix3_vectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut);
ErrorStatus matrix3_transposeVectorMultiply(Matrixf * mat, Vectorf * vecIn, Vectorf * vecOut);
ErrorStatus matrix3_MatrixMultiply(Matrixf * matA, Matrixf * matB, Matrixf * matC);
ErrorStatus matrix3_transpose(Matrixf * matA, Matrixf * matB);
ErrorStatus matrix3_copy(Matrixf * matA, Matrixf * matB);
ErrorStatus matrix3_normalizeOrthogonalizeMatrix(Matrixf * rotMatrix, float32_t maxError);

#endif /* MYMATH_MATRIX3_H_ */
