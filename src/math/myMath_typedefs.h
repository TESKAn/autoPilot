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
	uint16_t ostanek;
}__attribute__((aligned(4),packed)) Vectorui16;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t ostanek;
}__attribute__((aligned(4),packed)) Vectori16;

// a,b,c are matrix rows
typedef struct
{
	Vectorf a;
	Vectorf b;
	Vectorf c;
}__attribute__((aligned(4),packed)) Matrixf;

typedef struct
{
	uint32_t limit;			// PID output limit reached

	float32_t p;			// P part
	float32_t i;			// I part
	float32_t d;			// D part

	float32_t s;			// Sum output, p+i+d

	float32_t im;			// Integral sum of errors, saturate

	float32_t outMax;		//
	float32_t outMin;		//

	float32_t em;			// Previous error
	float32_t ed;			// Previous derivative
	float32_t Kp;			// *PID_K_MULTI
	float32_t Ki;			// *PID_K_MULTI
	float32_t Kd;			// *PID_K_MULTI

}__attribute__((aligned(4),packed)) myMath_PID;

typedef struct
{
	myMath_PID x;
	myMath_PID y;
	myMath_PID z;
}__attribute__((aligned(4),packed)) myMath_PID3;

typedef struct
{
	float32_t filter_acc;
	float32_t filter_result;
	float32_t window;
}__attribute__((aligned(4),packed)) myMath_filter;

typedef struct
{
	myMath_filter X;
	myMath_filter Y;
	myMath_filter Z;
	Vectorf result;
}__attribute__((aligned(4),packed)) myMath_filter3;

#endif /* MYMATH_TYPEDEFS_H_ */
