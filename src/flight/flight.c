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
	data->f32MaxHoverSpeed = RC_DEFAULT_HOVER_MAX_SPEED;
	data->f32NacelleTiltSpeed = RC_DEFAULT_TILT_SPEED;
	data->f32NacellePlaneTransitionTilt = RC_NACELLE_TRANSITION_TILT;
	data->f32NacelleHoverTransitionTilt = RC_NACELLE_HOVERTRANSITION_TILT;

	// Set nacelle tilt state
	data->f32NacelleCommonTilt = RC_DEFAULT_NACELLE_TILT;
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

	RCInputs->SCALES.f32AileronScale = RC_IN_DEFAULT_SCALE_AILERON;
	RCInputs->SCALES.f32ElevatorScale = RC_IN_DEFAULT_SCALE_ELEVATOR;
	RCInputs->SCALES.f32RudderScale = RC_IN_DEFAULT_SCALE_RUDDER;
	RCInputs->SCALES.f32ThrottleScale = RC_IN_DEFAULT_SCALE_THROTTLE;
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
			FCFlightData->ORIENTATION_REQUIRED_H.f32Pitch += RCInputs->RC_THROTTLE * RCInputs->SCALES.f32ThrottleScale;
		}
		// Check elevator input
		// Elevator -> go up/down
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_ELEVATOR))
		{
			FCFlightData->ORIENTATION_REQUIRED_H.f32Altitude += RCInputs->RC_ELEVATOR * RCInputs->SCALES.f32ElevatorScale;
		}
		// Check aileron input
		// Aileron -> roll left/right
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_AILERON))
		{
			FCFlightData->ORIENTATION_REQUIRED_H.f32Roll += RCInputs->RC_AILERON * RCInputs->SCALES.f32AileronScale;
		}
		// Check rudder input
		// Rudder - yaw left/right
		if(RC_IN_ZERO_VAL_OFFSET < math_absF(RCInputs->RC_RUDDER))
		{
			FCFlightData->ORIENTATION_REQUIRED_H.f32Yaw += RCInputs->RC_RUDDER * RCInputs->SCALES.f32RudderScale;
		}
		// Check gear/gyro
		// Vehicle mode select
		if(0 < RCInputs->RC_GEAR_GYRO)
		{
			// Go to plane mode
			RC_Flags.bits.TRANSITION = 1;
		}
		else
		{
			// Transition not finished, abort
			RC_Flags.bits.TRANSITION = 0;
		}

		// Check RC_FLAPS_PITCH
		// Arm unit
		if(0 < RCInputs->RC_FLAPS_PITCH)
		{
			// Go to plane mode
			RC_Flags.bits.ARMED = 1;
		}
		else
		{
			// Transition not finished, abort
			RC_Flags.bits.ARMED = 0;
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
		else
		{
			// Transition not finished, abort
			RC_Flags.bits.TRANSITION = 0;
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
				data->ui32FlightStateMachine = FLIGHT_STABILIZE_TRANSITION;
			}
			// Run stabilising algorithm
			flight_stabilizeHover(data);
			break;
		}
		case FLIGHT_STABILIZE_PLANE:
		{
			// Check - do we go for transition?
			if(RC_Flags.bits.TRANSITION)
			{
				data->ui32FlightStateMachine = FLIGHT_STABILIZE_TRANSITION;
			}

			break;
		}
		case FLIGHT_STABILIZE_TRANSITION:
		{
			// Tilt state machine
			switch(data->ui32FlightTransitionState)
			{
				case FLIGHT_TILT_START:
				{
					// Decide for next step
					if(RC_Flags.bits.HOVER)
					{
						// Go to plane mode
						data->ui32FlightTransitionState = FLIGHT_TILT_P;
					}
					else if(RC_Flags.bits.PLANE)
					{
						// Go to hover mode
						data->ui32FlightTransitionState = FLIGHT_TILT_H;
					}
					break;
				}
				case FLIGHT_TILT_P:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Yes
						// Tilt nacelles to transition tilt - from 90 deg to transition tilt
						data->f32NacelleCommonTilt -= data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt - data->f32NacellePlaneTransitionTilt) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to next step
							data->ui32FlightTransitionState = FLIGHT_TILT_TILTED_P;
						}
						data->f32NacelleTilt_FL -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM -= data->f32NacelleTiltSpeed;
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_P;
					}

					break;
				}
				case FLIGHT_TILT_H:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Yes
						// Tilt nacelles to transition tilt - from 0 deg to transition tilt
						data->f32NacelleCommonTilt += data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt - data->f32NacelleHoverTransitionTilt) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to next step
							data->ui32FlightTransitionState = FLIGHT_TILT_TILTED_H;
						}
						data->f32NacelleTilt_FL += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM += data->f32NacelleTiltSpeed;
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_H;
					}
					break;
				}
				case FLIGHT_TILT_TILTED_P:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Wait for speed to be over transition speed
						if(data->ORIENTATION.f32Speed >= data->f32MinPlaneSpeed)
						{
							// Next step - tilt nacelles to 0 deg
							data->ui32FlightTransitionState = FLIGHT_TILT_SPEED_OK_P;
						}
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_P;
					}
					break;
				}
				case FLIGHT_TILT_TILTED_H:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Wait for speed to be under transition speed
						if(data->ORIENTATION.f32Speed <= data->f32MaxHoverSpeed)
						{
							// Next step - tilt nacelles to 0 deg
							data->ui32FlightTransitionState = FLIGHT_TILT_SPEED_OK_H;
						}
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_H;
					}
					break;
				}
				case FLIGHT_TILT_SPEED_OK_P:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Tilt nacelles to 0 deg for flight
						data->f32NacelleCommonTilt -= data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to end
							data->ui32FlightTransitionState = FLIGHT_TILT_END_P;
						}
						data->f32NacelleTilt_FL -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM -= data->f32NacelleTiltSpeed;
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_TILT_P;
					}
					break;
				}
				case FLIGHT_TILT_SPEED_OK_H:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Tilt nacelles to 90 deg for hover
						data->f32NacelleCommonTilt += data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt - 90.0f) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to end
							data->ui32FlightTransitionState = FLIGHT_TILT_END_H;
						}
						data->f32NacelleTilt_FL += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM += data->f32NacelleTiltSpeed;
					}
					else
					{
						// Transition aborted
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_TILT_H;
					}
					break;
				}
				case FLIGHT_TILT_END_P:
				{
					// All done, end transition
					// Reset tilt state
					data->ui32FlightTransitionState = FLIGHT_TILT_START;
					// Set state machine to flight
					data->ui32FlightStateMachine = FLIGHT_STABILIZE_PLANE;
					// End transition
					RC_Flags.bits.TRANSITION = 0;
					// Go to plane mode
					RC_Flags.bits.PLANE = 1;
					// Go out of hover mode
					RC_Flags.bits.HOVER = 0;
					break;
				}
				case FLIGHT_TILT_END_H:
				{
					// All done, end transition
					// Reset tilt state
					data->ui32FlightTransitionState = FLIGHT_TILT_START;
					// Set state machine to flight
					data->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
					// End transition
					RC_Flags.bits.TRANSITION = 0;
					// Go to plane mode
					RC_Flags.bits.PLANE = 0;
					// Go out of hover mode
					RC_Flags.bits.HOVER = 1;
					break;
				}
				case FLIGHT_TILT_ABORT_TILT_P:
				{
					if(RC_Flags.bits.TRANSITION)
					{
						// go back to transition
						data->ui32FlightTransitionState = FLIGHT_TILT_SPEED_OK_P;
					}
					else
					{
						// Tilt back
						// Tilt nacelles to hover transition tilt
						data->f32NacelleCommonTilt += data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt - data->f32NacelleHoverTransitionTilt) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to next step
							data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_TILTED_P;
						}
						data->f32NacelleTilt_FL += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR += data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM += data->f32NacelleTiltSpeed;
					}
					break;
				}
				case FLIGHT_TILT_ABORT_TILT_H:
				{
					if(RC_Flags.bits.TRANSITION)
					{
						// go back to transition
						data->ui32FlightTransitionState = FLIGHT_TILT_SPEED_OK_H;
					}
					else
					{
						// Tilt back
						// Tilt nacelles to plane transition tilt
						data->f32NacelleCommonTilt -= data->f32NacelleTiltSpeed;
						// Is difference small enough?
						if(math_absF(data->f32NacelleCommonTilt - data->f32NacellePlaneTransitionTilt) < data->f32NacelleTiltSpeed)
						{
							// Yes it is, go to next step
							data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_TILTED_H;
						}
						data->f32NacelleTilt_FL -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_FR -= data->f32NacelleTiltSpeed;
						data->f32NacelleTilt_BM -= data->f32NacelleTiltSpeed;
					}
					break;
				}
				case FLIGHT_TILT_ABORT_TILTED_P:
				{
					if(RC_Flags.bits.TRANSITION)
					{
						// Go back to transition
					}
					else
					{
						// Wait for speed to drop below level
						if(data->ORIENTATION.f32Speed < data->f32MaxHoverSpeed)
						{
							// Next step - tilt nacelles to 90 deg
							data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_P;
						}
					}
					break;
				}
				case FLIGHT_TILT_ABORT_TILTED_H:
				{
					if(RC_Flags.bits.TRANSITION)
					{
						// Go back to transition
					}
					else
					{
						// Wait for speed to go above plane level
						if(data->ORIENTATION.f32Speed > data->f32MinPlaneSpeed)
						{
							// Next step - tilt nacelles to 0 deg
							data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_SPEED_OK_H;
						}
					}
					break;
				}
				case FLIGHT_TILT_ABORT_SPEED_OK_P:
				{
					// Tilt nacelles to 90 deg
					data->f32NacelleCommonTilt += data->f32NacelleTiltSpeed;
					// Is difference small enough?
					if(math_absF(data->f32NacelleCommonTilt - 90.0f) < data->f32NacelleTiltSpeed)
					{
						// Yes it is, go to next step
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_END_P;
					}
					data->f32NacelleTilt_FL += data->f32NacelleTiltSpeed;
					data->f32NacelleTilt_FR += data->f32NacelleTiltSpeed;
					data->f32NacelleTilt_BM += data->f32NacelleTiltSpeed;
					break;
				}
				case FLIGHT_TILT_ABORT_SPEED_OK_H:
				{
					// Tilt nacelles to 0 deg
					data->f32NacelleCommonTilt -= data->f32NacelleTiltSpeed;
					// Is difference small enough?
					if(math_absF(data->f32NacelleCommonTilt) < data->f32NacelleTiltSpeed)
					{
						// Yes it is, go to next step
						data->ui32FlightTransitionState = FLIGHT_TILT_ABORT_END_H;
					}
					data->f32NacelleTilt_FL -= data->f32NacelleTiltSpeed;
					data->f32NacelleTilt_FR -= data->f32NacelleTiltSpeed;
					data->f32NacelleTilt_BM -= data->f32NacelleTiltSpeed;
					break;
				}
				case FLIGHT_TILT_ABORT_END_P:
				{
					// All done, end transition
					// Reset tilt state
					data->ui32FlightTransitionState = FLIGHT_TILT_START;
					// Set state machine to flight
					data->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
					// End transition
					RC_Flags.bits.TRANSITION = 0;
					// Go to plane mode
					RC_Flags.bits.PLANE = 0;
					// Go out of hover mode
					RC_Flags.bits.HOVER = 1;
					break;
				}
				case FLIGHT_TILT_ABORT_END_H:
				{
					// All done, end transition
					// Reset tilt state
					data->ui32FlightTransitionState = FLIGHT_TILT_START;
					// Set state machine to flight
					data->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
					// End transition
					RC_Flags.bits.TRANSITION = 0;
					// Go to plane mode
					RC_Flags.bits.PLANE = 1;
					// Go out of hover mode
					RC_Flags.bits.HOVER = 0;
					break;
				}
				default:
				{
					data->ui32FlightTransitionState = FLIGHT_TILT_START;
				}
			}
			break;
		}


		default:
		{
			data->ui32FlightStateMachine = FLIGHT_IDLE;
			break;
		}
	}
}

// Stabilize hover according to data
void flight_stabilizeHover(FLIGHT_CORE * FCFlightData)
{
//	float32_t f32Temp = 0;
	float32_t f32Error = 0.0f;
	// Use PID regulator to stabilize vehicle
	// Stabilize roll - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED_H.f32Roll - FCFlightData->ORIENTATION.f32Roll;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDRoll);

	// Stabilize pitch - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED_H.f32Pitch - FCFlightData->ORIENTATION.f32Pitch;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDPitch);

	// Stabilize yaw - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED_H.f32Yaw - FCFlightData->ORIENTATION.f32Yaw;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDYaw);

	// Run altitude PID
	f32Error = FCFlightData->ORIENTATION_REQUIRED_H.f32Altitude - FCFlightData->ORIENTATION.f32Altitude;
	math_PID(f32Error, 0.01f, &FCFlightData->PIDAltitude);
}

// Hardware specific function - decode flight commands to servo commands
void flight_decodeServos(FLIGHT_CORE * FCFlightData, RCDATA * RCValues)
{
	// Next depends on current mode
	if(RC_Flags.bits.PLANE)
	{

	}
	else if(RC_Flags.bits.HOVER)
	{
		// We are in hover mode, all maneuvering is done with motors, flaps/ailerons to middle
		RCValues->RC_AILERON_L = RCValues->RC_AILERON_L_MID;
		RCValues->RC_AILERON_R = RCValues->RC_AILERON_R_MID;

		RCValues->RC_MOTOR_FL = FCFlightData->PIDPitch.s + FCFlightData->PIDRoll.s + FCFlightData->PIDAltitude.s;

		RCValues->RC_MOTOR_FR = FCFlightData->PIDPitch.s - FCFlightData->PIDRoll.s + FCFlightData->PIDAltitude.s;

		RCValues->RC_MOTOR_BM = -FCFlightData->PIDPitch.s + FCFlightData->PIDAltitude.s;

		// Controll yaw with nacelle roll
		RCValues->RC_NACELLE_BR = FCFlightData->PIDYaw.s;

	}
	else
	{

	}
}

