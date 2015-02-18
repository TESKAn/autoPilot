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
void flight_init(FLIGHT_CORE *data, RCDATA * RCInputs)
{
	// Set initial state
	data->ui32FlightStateMachine = FLIGHT_IDLE;
	data->f32MinPlaneSpeed = RC_DEFAULT_PLANE_MIN_SPEED;
	data->f32NacelleTiltSpeed = RC_DEFAULT_TILT_SPEED;
	// Set nacelle tilt state
	data->f32NacelleTilt_FL = RC_DEFAULT_NACELLE_TILT;
	data->f32NacelleTilt_FR = RC_DEFAULT_NACELLE_TILT;
	data->f32NacelleTilt_BM = RC_DEFAULT_NACELLE_TILT;

	RCInputs->PWMIN_1_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_2_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_3_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_4_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_5_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_6_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_7_MID = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMIN_8_MID = RC_IN_DEFAULT_MIDPOINT;

	RCInputs->PWMOUT_Offset_1 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_2 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_3 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_4 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_5 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_6 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_7 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_8 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_9 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_10 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_11 = RC_IN_DEFAULT_MIDPOINT;
	RCInputs->PWMOUT_Offset_12 = RC_IN_DEFAULT_MIDPOINT;
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
		// Throttle -> pitch forward/backward
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_THROTTLE))
		{
			FCFlightData->ORIENTATION_REQUIRED_Q.f32Pitch = RCInputs->RC_THROTTLE;
		}
		// Check elevator input
		// Elevator -> go up/down
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_ELEVATOR))
		{
			FCFlightData->ORIENTATION_REQUIRED_Q.f32AltitudeChange = RCInputs->RC_ELEVATOR;
		}
		// Check aileron input
		// Aileron -> roll left/right
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_AILERON))
		{
			FCFlightData->ORIENTATION_REQUIRED_Q.f32Roll = RCInputs->RC_AILERON;
		}
		// Check rudder input
		// Rudder - yaw left/right
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_RUDDER))
		{
			FCFlightData->ORIENTATION_REQUIRED_Q.f32Yaw = RCInputs->RC_RUDDER;
		}
		// Check gear/gyro
		// Vehicle mode select
		if(0 < RCInputs->RC_GEAR_GYRO)
		{
			// Go to plane mode
			RC_Flags.bits.TRANSITION = 1;
		}



	}// end if
	else if(RC_Flags.bits.PLANE)
	{
		// Else we are in plane mode

		// Check gear/gyro
		// Vehicle mode select
		if(0 > RCInputs->RC_GEAR_GYRO)
		{
			// Go to hover mode
			RC_Flags.bits.TRANSITION = 1;
		}

	}// end else if
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
		case FLIGHT_STABILIZE_HOVER:
		{
			// Check - do we go for transition?
			if(RC_Flags.bits.TRANSITION)
			{
				// Transition from one mode to other mode
				// Check if we have enough speed
				if(data->ORIENTATION.f32Speed > data->f32MinPlaneSpeed)
				{
					// We are in plane mode
					RC_Flags.bits.TRANSITION = 0;
					RC_Flags.bits.PLANE = 1;
					data->ui32FlightStateMachine = FLIGHT_STABILIZE_PLANE;
				} // end if
				else
				{
					// We are in transition, pick up speed and keep vehicle level
					// Change nacelle tilt toward 0 deg
					data->f32NacelleTilt_FL

				} // end else



			} // end if

			break;
		}
		case FLIGHT_STABILIZE_PLANE:
		{

			break;
		}
		case FLIGHT_STABILIZE_TRANSITION:
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
