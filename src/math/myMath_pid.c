/*
 * myMath_pid.c
 *
 *  Created on: Sep 5, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "allinclude.h"
#include "myMath_typedefs.h"
#include "myMath_vec3.h"
#include "myMath.h"
#include "myMath_pid.h"

myMath_PID3 _gyroErrorPID;

myMath_PID math_PIDInit(float32_t kp, float32_t ki, float32_t kd)
{
	myMath_PID PID;
	math_PIDReset(&PID);
	PID.Kp = kp;
	PID.Ki = ki;
	PID.Kd = kd;

	PID.errIMax = 100;
	PID.errIMin = -100;

	PID.errMax = 100;
	PID.errMin = -100;

	PID.outMax = 100;
	PID.outMin = -100;

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
	PID->p = 0;
	PID->i = 0;
	PID->d = 0;
	PID->s = 0;
	PID->im = 0;
	PID->em = 0;
	PID->ed = 0;
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

ErrorStatus math_PID(float32_t error, float32_t dt, myMath_PID * data)
{
	ErrorStatus status = ERROR;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	// Saturate error
	if(error > data->errMax) error = data->errMax;
	else if(error < data->errMin) error = data->errMin;
	// Calculate p
	data->p = error * data->Kp;
	// Calculate I
	// Add error * dt to integral
	data->im = data->im + (error * dt);
	// Saturate I
	if(data->im > data->errIMax) data->im = data->errIMax;
	else if(data->im < data->errIMin) data->im = data->errIMin;
	// Calculate I as im*Ki
	data->i = data->im * data->Ki;

	// Calculate d
	// First de
	data->ed = data->em - error;
	// Then de/dt
	data->ed = data->ed / dt;
	// Then d
	data->d = data->ed * data->Kd;

	// Store error
	data->em = error;

	// Calculate PID output
	data->s = data->p + data->i + data->d;

	// Saturate output
	if(data->s > data->outMax) data->s = data->outMax;
	else if(data->s < data->outMin) data->s = data->outMin;

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
	}

	return status;
}

ErrorStatus math_PID3(Vectorf * error, float32_t dt, myMath_PID3 * PID)
{
	ErrorStatus status = ERROR;
	// Set FPU exception bit to 0
	FPU_EXCEPTION = 0;

	// For each PID, call math_PID
	math_PID(error->x, dt, &PID->x);
	math_PID(error->y, dt, &PID->y);
	math_PID(error->z, dt, &PID->z);

	// Check if FPU result is OK
	if(!FPU_EXCEPTION)
	{
		status = SUCCESS;
	}

	return status;
}
