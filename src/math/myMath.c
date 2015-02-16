/*
 * myMath.c
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "myMath_typedefs.h"
#include "myMath_vec3.h"
#include "myMath.h"

// Calculate fast inverse square root of a number
float32_t math_fastInverseRoot(float32_t num)
{
	float32_t result = 0;
	union { float32_t f; uint32_t u; } y = {num};
	y.u = (uint32_t)(0x5F1FFF77) - (y.u >> 1);
	result = 0.703974056f * y.f * (2.38919526f - (num * y.f * y.f));
	return result;
}

// Calculate float absolute value
float32_t math_absF(float32_t val)
{
	if(0 < val) return val;
	else return -val;
}
