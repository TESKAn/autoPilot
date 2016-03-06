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

ErrorStatus math_PIDInit(myMath_PID* PID, float32_t kp, float32_t ki, float32_t kd, float32_t outMin, float32_t outMax)
{
	math_PIDReset(PID);
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;

	PID->limit = 0;

	PID->outMax = outMax;
	PID->outMin = outMin;

	return SUCCESS;
}

ErrorStatus math_PID3Init(myMath_PID3* PID, float32_t kp, float32_t ki, float32_t kd, float32_t outMin, float32_t outMax)
{
	math_PIDInit(&PID->x, kp, ki, kd, outMin, outMax);
	math_PIDInit(&PID->y, kp, ki, kd, outMin, outMax);
	math_PIDInit(&PID->z, kp, ki, kd, outMin, outMax);
	return SUCCESS;
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


	// Calculate p
	data->p = error * data->Kp;
	// Calculate I
	if(0 == data->limit)
	{
		// Add error * dt to integral
		data->im = data->im + (error * dt);
	}
	else
	{
		// Check what error does to integral
		if(0.0f < data->im)
		{
			if(0.0f > error)
			{
				data->im = data->im + (error * dt);
			}
		}
		else
		{
			if(0.0f < error)
			{
				data->im = data->im + (error * dt);
			}
		}
	}
	// Calculate I as im*Ki
	data->i = data->im * data->Ki;

	// Calculate d
	// First de
	data->ed = error - data->em;
	// Then de/dt
	// Check that dt is not 0
	if(dt != 0)
	{
		// Then dt
		data->ed = data->ed * data->Kd;
		data->d = data->ed / dt;
	}
	else
	{
		data->ed = 0;
		data->d = 0;
	}

	// Store error
	data->em = error;

	// Calculate PID output
	data->s = data->p + data->i + data->d;

	// Saturate output
	if(data->s > data->outMax)
	{
		data->s = data->outMax;
		data->limit = 1;
	}
	else if(data->s < data->outMin)
	{
		data->s = data->outMin;
		data->limit = 1;
	}
	else
	{
		data->limit = 0;
	}

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
