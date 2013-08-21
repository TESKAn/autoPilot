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

myMath_PID math_PIDInit(float32_t kp, float32_t ki, float32_t kd)
{
	myMath_PID PID;
	PID.e_1 = 0;
	PID.i = 0;
	PID.result = 0;
	PID.kp = kp;
	PID.ki = ki;
	PID.kd = kd;

	return PID;
}

myMath_PID3 math_PID3Init(float32_t kp, float32_t ki, float32_t kd)
{
	myMath_PID3 PID;
	PID.x = math_PIDInit(kp, ki, kd);
	PID.y = math_PIDInit(kp, ki, kd);
	PID.z = math_PIDInit(kp, ki, kd);
	return PID;
}

ErrorStatus math_PIDReset(myMath_PID * PID)
{
	ErrorStatus status = ERROR;
	// Reset PID variables to zero
	PID->e_1 = 0;
	PID->i = 0;
	PID->result = 0;
	status = SUCCESS;
	return status;
}

ErrorStatus math_PID3Reset(myMath_PID3 * PID)
{
	ErrorStatus status = ERROR;
	// Reset PID variables to zero
	math_PIDReset(&PID->x);
	math_PIDReset(&PID->y);
	math_PIDReset(&PID->z);
	status = SUCCESS;
	return status;
}

ErrorStatus math_PID(float32_t e, float32_t dt, myMath_PID * PID)
{
	ErrorStatus status = ERROR;
	// Set variables
	float32_t p = 0;
	float32_t i = 0;
	float32_t d = 0;
	// Calculate proportional part
	p = e * PID->kp;
	// Calculate integral and integral part
	i = e * dt;
	PID->i = PID->i + i;
	i = PID->i * PID->ki;
	// Calculate differential part and store current error
	d = e - PID->e_1;
	d = d / dt;
	d = d * PID->kd;
	// Calculate result
	PID->result = p + i + d;
	status = SUCCESS;
	return status;
}

ErrorStatus math_PID3(Vectorf * error, float32_t dt, myMath_PID3 * PID)
{
	ErrorStatus status = ERROR;
	// For each PID, call math_PID
	math_PID(error->x, dt, &PID->x);
	math_PID(error->y, dt, &PID->y);
	math_PID(error->z, dt, &PID->z);

	status = SUCCESS;
	return status;
}
