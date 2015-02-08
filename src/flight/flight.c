/*
 * flight.c
 *
 *  Created on: 8. feb. 2015
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "syscalls.h"
#include "arm_math.h"
#include "flight_typedefs.h"
#include "flight.h"

#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "math/myMath_pid.h"


// Init flight variables
void flight_init(FLIGHT_CORE *data)
{
	data->ui32FlightStateMachine = FLIGHT_IDLE;
}

// Main flight state machine
void flight_checkStates(FLIGHT_CORE *data)
{

	switch(data->ui32FlightStateMachine)
	{
		case FLIGHT_IDLE:
		{
			break;
		}
		case FLIGHT_STABILIZE:
		{
			break;
		}


		default:
		{
			data->ui32FlightStateMachine = FLIGHT_IDLE;
			break;
		}
	}


}
