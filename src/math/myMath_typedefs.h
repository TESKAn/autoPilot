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

typedef struct
{
	Vectorf a;
	Vectorf b;
	Vectorf c;
}__attribute__((aligned(4),packed)) Matrixf;

typedef struct
{
	float32_t p;			// P part
	float32_t i;			// I part
	float32_t d;			// D part

	float32_t s;			// Sum output, p+i+d

	float32_t im;			// Integral sum of errors, saturated

	float32_t errIMax;		// Integral max sum
	float32_t errIMin;		// Integral min sum

	float32_t outMax;		// *PID_K_MULTI
	float32_t outMin;		// *PID_K_MULTI

	float32_t em;			// Previous error
	float32_t ed;			// Previous derivative
	float32_t Kp;			// *PID_K_MULTI
	float32_t Ki;			// *PID_K_MULTI
	float32_t Kd;			// *PID_K_MULTI

	float32_t errMax;
	float32_t errMin;

}__attribute__((aligned(4),packed)) myMath_PID;

typedef struct
{
	myMath_PID x;
	myMath_PID y;
	myMath_PID z;
}__attribute__((aligned(4),packed)) myMath_PID3;

#endif /* MYMATH_TYPEDEFS_H_ */
