/*
 * myMath_typedefs.h
 *
 *  Created on: Aug 21, 2013
 *      Author: Jure
 */

#ifndef MYMATH_TYPEDEFS_H_
#define MYMATH_TYPEDEFS_H_

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

typedef struct
{
	Vectorf a;
	Vectorf b;
	Vectorf c;
}__attribute__((aligned(4),packed)) Matrixf;

typedef struct
{
	float32_t i;
	float32_t kp;
	float32_t ki;
	float32_t kd;
	float32_t e_1;
	float32_t result;
}__attribute__((aligned(4),packed)) myMath_PID;

typedef struct
{
	myMath_PID x;
	myMath_PID y;
	myMath_PID z;
}__attribute__((aligned(4),packed)) myMath_PID3;

#endif /* MYMATH_TYPEDEFS_H_ */
