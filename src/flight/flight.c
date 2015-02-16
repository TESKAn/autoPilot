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
#include "math/myMath.h"

RCFlag RC_Flags;


// Init flight variables
void flight_init(FLIGHT_CORE *data)
{
	// Set initial state
	data->ui32FlightStateMachine = FLIGHT_IDLE;
}

// Decode what RC commands want
void flight_checkRCInputs(RCDATA * RCInputs, FLIGHT_CORE * FCFlightData)
{
	// Decode RC inputs and turn them into flight data commands
	// RC input
	// Recalculate values - remove offset from inputs
	RCInputs->PWMIN_1_Zero = (float32_t)RCInputs->PWMIN_1 - RCInputs->PWMIN_1_MID;
	RCInputs->PWMIN_2_Zero = (float32_t)RCInputs->PWMIN_2 - RCInputs->PWMIN_2_MID;
	RCInputs->PWMIN_3_Zero = (float32_t)RCInputs->PWMIN_3 - RCInputs->PWMIN_3_MID;
	RCInputs->PWMIN_4_Zero = (float32_t)RCInputs->PWMIN_4 - RCInputs->PWMIN_4_MID;
	RCInputs->PWMIN_5_Zero = (float32_t)RCInputs->PWMIN_5 - RCInputs->PWMIN_5_MID;
	RCInputs->PWMIN_6_Zero = (float32_t)RCInputs->PWMIN_6 - RCInputs->PWMIN_6_MID;
	RCInputs->PWMIN_7_Zero = (float32_t)RCInputs->PWMIN_7 - RCInputs->PWMIN_7_MID;
	RCInputs->PWMIN_8_Zero = (float32_t)RCInputs->PWMIN_8 - RCInputs->PWMIN_8_MID;

	// Design is for VTOL, so first decide if we are hovering or flying
	if(RC_Flags.bits.HOVER)
	{
		// We are in hover mode
		// Check throttle input
		// Check if absolute value is over some limit for deadzone
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_THROTTLE))
		{
			FCFlightData->ORIENTATION_REQUIRED_Q.f32AltitudeChange = RCInputs->RC_THROTTLE;
		}





	}// if
	else
	{
		// Else we are in plane mode
	}// else

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
