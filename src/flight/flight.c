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

uint16_t ui16CheckStatesDelay = 0;

// Angle deviation for level and vtol
#define ANGLE_DEV_LEVEL_P		2.0f
#define ANGLE_DEV_LEVEL_N		-2.0f

#define ANGLE_DEV_VTOL_P		92.0f
#define ANGLE_DEV_VTOL_N		88.0f

// Max nacelle roll
#define MAX_NACELLE_ROLL		100.0f



// Init flight variables
void flight_init(FLIGHT_CORE *FCFlightData, RCDATA * RCValues)
{
	int i = 0;
	// Set initial state
	FCFlightData->ui32FlightStateMachine = FLIGHT_IDLE;

	FCFlightData->ui32FlightInitState = FINIT_IDLE;

	// Init PIDs
	math_PIDInit(&FCFlightData->PIDPitch, 1.1f, 0.1f, 0.0f, -0.50f, 0.50f);
	math_PIDInit(&FCFlightData->PIDRoll, 1.1f, 0.1f, 0.0f, -0.50f, 0.50f);
	math_PIDInit(&FCFlightData->PIDYaw, 1.1f, 0.1f, 0.0f, -5.0f, 5.0f);
	math_PIDInit(&FCFlightData->PIDAltitude, 1.1f, 0.1f, 0.0f, 0.0f, 1.0f);
	math_PIDInit(&FCFlightData->PIDSpeed, 1.1f, 0.1f, 0.0f, 0.0f, 1.0f);

	FCFlightData->ORIENTATION_REQUIRED.f32Roll = 0.0f;
	FCFlightData->ORIENTATION_REQUIRED.f32Pitch = 0.0f;
	FCFlightData->ORIENTATION_REQUIRED.f32Yaw = 0.0f;
	FCFlightData->ORIENTATION_REQUIRED.f32Altitude = 0.0f;
	FCFlightData->ORIENTATION_REQUIRED.f32Speed = 0.0f;
	FCFlightData->ORIENTATION_REQUIRED.f32Power = 0.0f;

	FCFlightData->ORIENTATION_LIMITS.f32RollLimit = 10.0f;
	FCFlightData->ORIENTATION_LIMITS.f32PitchLimit = 10.0f;
	FCFlightData->ORIENTATION_LIMITS.f32YawLimit = 360.0f;

	FCFlightData->f32MinPlaneSpeed = RC_DEFAULT_PLANE_MIN_SPEED;
	FCFlightData->f32MaxHoverSpeed = RC_DEFAULT_HOVER_MAX_SPEED;
	FCFlightData->f32NacelleTiltSpeed = RC_DEFAULT_TILT_SPEED;
	FCFlightData->f32NacellePlaneTransitionTilt = RC_NACELLE_TRANSITION_TILT;
	FCFlightData->f32NacelleHoverTransitionTilt = RC_NACELLE_HOVERTRANSITION_TILT;

	// Set nacelle tilt state
	FCFlightData->f32NacelleCommonTilt = RC_DEFAULT_NACELLE_TILT;
	FCFlightData->f32NacelleTilt_FL = RC_DEFAULT_NACELLE_TILT;
	FCFlightData->f32NacelleTilt_FR = RC_DEFAULT_NACELLE_TILT;
	FCFlightData->f32NacelleTilt_R = RC_DEFAULT_NACELLE_TILT;
	// Set zero angle values
	FCFlightData->TILT_SERVOS.f32AllowedPositionDeviation = 20.0f;	// ~2 deg
	FCFlightData->TILT_SERVOS.FR.f32ServoZero = NACELLE_FR_ZERO;
	FCFlightData->TILT_SERVOS.FL.f32ServoZero = NACELLE_FL_ZERO;
	FCFlightData->TILT_SERVOS.R.f32ServoZero = NACELLE_R_ZERO;

	for(i=0; i < 12; i++)
	{
		RCValues->ch[i].PWMIN_MID = RC_IN_DEFAULT_MIDPOINT;
		RCValues->ch[i].PWMOUT_Offset = RC_IN_DEFAULT_MIDPOINT;

		RCValues->ch[i].PWMMin = 1200;
		RCValues->ch[i].PWMMax = 1800;
		RCValues->ch[i].PWMDiff = 600;
		RCValues->ch[i].PWMOUT = 1500;
	}
	// Set default PWM outputs

	// Gear down
	RCValues->ch[RC_GEAR].PWMOUT = 2000;

	// Ailerons to midpoint
	RCValues->ch[RC_AILERON_L].PWMOUT = 1500;
	RCValues->ch[RC_AILERON_R].PWMOUT = 1500;
	// Motors to zero
	RCValues->ch[RC_MOTOR_FL].PWMOUT = 1000;
	RCValues->ch[RC_MOTOR_FR].PWMOUT = 1000;
	RCValues->ch[RC_MOTOR_R].PWMOUT = 1000;


	// Nacelle tilt to midpoint
	RCValues->ch[RC_MOTOR_R_TILT].PWMOUT = 1500;


	RCValues->SCALES.f32AileronScale = RCValues->ch[RC_AILERON].PWMDiff / FCFlightData->ORIENTATION_LIMITS.f32RollLimit;
	RCValues->SCALES.f32ElevatorScale = RCValues->ch[RC_ELEVATOR].PWMDiff / FCFlightData->ORIENTATION_LIMITS.f32PitchLimit;

	RCValues->SCALES.f32RudderScale = RC_IN_DEFAULT_SCALE_RUDDER;
	RCValues->SCALES.f32ThrottleScale = RC_IN_DEFAULT_SCALE_THROTTLE;
}

// Decode what RC commands want
void flight_checkRCInputs(FLIGHT_CORE * FCFlightData, RCDATA * RCValues)
{
	int i = 0;
	float32_t f32Temp;
	// Check state
	if(!RC_Flags.bits.ARMED)
	{
		// If not armed, check PWM inputs and adjust zero point for first four channels
		for(i=0; i < 4; i++)
		{
			RCValues->ch[i].PWMIN_MID = (float32_t)RCValues->ch[i].PWMIN;
		}
		// Store current altitude as zero altitude
		FCFlightData->ORIENTATION.f32ZeroAltitude = FCFlightData->ORIENTATION.f32Altitude;
	}

	// Check input limits
	for(i=0; i < 8; i++)
	{
		f32Temp = (float32_t)RCValues->ch[i].PWMIN;
		// Over max, under min
		if(f32Temp < RCValues->ch[i].PWMMin)
		{
			RCValues->ch[i].PWMMin = f32Temp;
			RCValues->ch[i].PWMDiff = RCValues->ch[i].PWMMax - RCValues->ch[i].PWMMin;
			// Recalculate scales
			if(RC_AILERON == i)
			{
				RCValues->SCALES.f32AileronScale = FCFlightData->ORIENTATION_LIMITS.f32RollLimit / RCValues->ch[RC_AILERON].PWMDiff;
			}
			else if(RC_ELEVATOR == i)
			{
				RCValues->SCALES.f32ElevatorScale = FCFlightData->ORIENTATION_LIMITS.f32PitchLimit / RCValues->ch[RC_ELEVATOR].PWMDiff;
			}
		}
		if(f32Temp > RCValues->ch[i].PWMMax)
		{
			RCValues->ch[i].PWMMax = f32Temp;
			RCValues->ch[i].PWMDiff = RCValues->ch[i].PWMMax - RCValues->ch[i].PWMMin;
			// Recalculate scales
			if(RC_AILERON == i)
			{
				RCValues->SCALES.f32AileronScale = FCFlightData->ORIENTATION_LIMITS.f32RollLimit / RCValues->ch[RC_AILERON].PWMDiff;
			}
			else if(RC_ELEVATOR == i)
			{
				RCValues->SCALES.f32ElevatorScale = FCFlightData->ORIENTATION_LIMITS.f32PitchLimit / RCValues->ch[RC_ELEVATOR].PWMDiff;
			}
		}
		// Calculate current value
		RCValues->ch[i].PWMIN_Zero = (float32_t)RCValues->ch[i].PWMIN - RCValues->ch[i].PWMIN_MID;
	}

	// Decode RC inputs and turn them into flight data commands

	// If hover
	if(RC_Flags.bits.HOVER)
	{
		// Turn throttle to limit for altitude reg.
		FCFlightData->PIDAltitude.outMax = RCValues->ch[RC_THROTTLE].PWMIN_Zero / RCValues->ch[i].PWMDiff;

		// And use it for altitude adjust
		FCFlightData->ORIENTATION_REQUIRED.f32Altitude += 0.0001f*(RCValues->ch[RC_THROTTLE].PWMIN_Zero - RCValues->ch[RC_THROTTLE].PWMIN_MID);
		// Limit altitude
		if(400.0f < FCFlightData->ORIENTATION_REQUIRED.f32Altitude)
		{
			FCFlightData->ORIENTATION_REQUIRED.f32Altitude = 400.0f;
		}
		else if(-100.0f > FCFlightData->ORIENTATION_REQUIRED.f32Altitude)
		{
			FCFlightData->ORIENTATION_REQUIRED.f32Altitude = -100.0f;
		}

		// Elevator controls pitch
		f32Temp = RCValues->ch[RC_ELEVATOR].PWMIN_Zero * RCValues->SCALES.f32ElevatorScale;
		FCFlightData->ORIENTATION_REQUIRED.f32Pitch = f32Temp;
		// Aileron controls roll
		f32Temp = RCValues->ch[RC_AILERON].PWMIN_Zero * RCValues->SCALES.f32AileronScale;
		FCFlightData->ORIENTATION_REQUIRED.f32Roll = f32Temp;
		// Rudder controls yaw
		f32Temp = RCValues->ch[RC_RUDDER].PWMIN_Zero * RCValues->SCALES.f32RudderScale;
		// Add to req
		FCFlightData->ORIENTATION_REQUIRED.f32Yaw += f32Temp;
		// Limit yaw to +/- 180
		if(180.0f < FCFlightData->ORIENTATION_REQUIRED.f32Yaw)
		{
			FCFlightData->ORIENTATION_REQUIRED.f32Yaw = 180.0f;
		}
		else if(-180.0f > FCFlightData->ORIENTATION_REQUIRED.f32Yaw)
		{
			FCFlightData->ORIENTATION_REQUIRED.f32Yaw = -180.0f;
		}
	}
}

// Main flight state machine
void flight_checkStates(FLIGHT_CORE *FCFlightData, RCDATA * RCValues)
{
	switch(FCFlightData->ui32FlightStateMachine)
	{
		case FLIGHT_IDLE:
		{
			break;
		}
		case FLIGHT_INIT:
		{
			switch(FCFlightData->ui32FlightInitState)
			{
				case FINIT_IDLE:
				{
					// Set motor PWMs to min
					RCValues->ch[RC_MOTOR_FL].PWMOUT = 1000;
					RCValues->ch[RC_MOTOR_FR].PWMOUT = 1000;
					RCValues->ch[RC_MOTOR_R].PWMOUT = 1000;
					// Reverse motor FR
					FCFlightData->MOTORS.FR.ui8ReverseRotation = 1;
					FCFlightData->MOTORS.FL.ui8ReverseRotation = 0;
					FCFlightData->MOTORS.R.ui8ReverseRotation = 0;
					// Enable motors
					FCFlightData->MOTORS.FR.ui8Enable = 1;
					FCFlightData->MOTORS.FL.ui8Enable = 1;
					FCFlightData->MOTORS.R.ui8Enable = 1;
					// Set to park
					FCFlightData->MOTORS.FR.ui8Park = 1;
					FCFlightData->MOTORS.FL.ui8Park = 1;
					FCFlightData->MOTORS.R.ui8Park = 1;
					// Mark measure low PWM time
					FCFlightData->MOTORS.FR.ui8MeasPWMMin = 1;
					FCFlightData->MOTORS.FL.ui8MeasPWMMin = 1;
					FCFlightData->MOTORS.R.ui8MeasPWMMin = 1;
					// Next state
					FCFlightData->ui32FlightInitState = FINIT_MEAS_PWM_MIN;
					ui16CheckStatesDelay = 50;
					break;
				}
				case FINIT_MEAS_PWM_MIN:
				{
					// Check if values are updated
					if(1 == FCFlightData->MOTORS.FR.ui8MeasuringPWMMin)
					{
						if(1 == FCFlightData->MOTORS.FL.ui8MeasuringPWMMin)
						{
							if(1 == FCFlightData->MOTORS.R.ui8MeasuringPWMMin)
							{
								if(0 == ui16CheckStatesDelay)
								{
									// Mark measure low PWM time
									FCFlightData->MOTORS.FR.ui8MeasPWMMin = 0;
									FCFlightData->MOTORS.FL.ui8MeasPWMMin = 0;
									FCFlightData->MOTORS.R.ui8MeasPWMMin = 0;
									// Next state
									FCFlightData->ui32FlightInitState = FINIT_WAIT_MEAS_PWMMIN;
									ui16CheckStatesDelay = 50;
								}
								else
								{
									ui16CheckStatesDelay--;
								}
							}
						}
					}

					break;
				}
				case FINIT_WAIT_MEAS_PWMMIN:
				{
					if(0 == FCFlightData->MOTORS.FR.ui8MeasuringPWMMin)
					{
						if(0 == FCFlightData->MOTORS.FL.ui8MeasuringPWMMin)
						{
							if(0 == FCFlightData->MOTORS.R.ui8MeasuringPWMMin)
							{
								FCFlightData->MOTORS.FR.ui8MeasPWMMax = 1;
								FCFlightData->MOTORS.FL.ui8MeasPWMMax = 1;
								FCFlightData->MOTORS.R.ui8MeasPWMMax = 1;
								// Set motor PWMs to max
								RCValues->ch[RC_MOTOR_FL].PWMOUT = 2000;
								RCValues->ch[RC_MOTOR_FR].PWMOUT = 2000;
								RCValues->ch[RC_MOTOR_R].PWMOUT = 2000;
								FCFlightData->ui32FlightInitState = FINIT_MEAS_PWM_MAX;
							}
						}
					}
					break;
				}
				case FINIT_MEAS_PWM_MAX:
				{
					if(1 == FCFlightData->MOTORS.FR.ui8MeasuringPWMMax)
					{
						if(1 == FCFlightData->MOTORS.FL.ui8MeasuringPWMMax)
						{
							if(1 == FCFlightData->MOTORS.R.ui8MeasuringPWMMax)
							{
								if(0 == ui16CheckStatesDelay)
								{
									FCFlightData->MOTORS.FR.ui8MeasPWMMax = 0;
									FCFlightData->MOTORS.FL.ui8MeasPWMMax = 0;
									FCFlightData->MOTORS.R.ui8MeasPWMMax = 0;
									// Next state
									FCFlightData->ui32FlightInitState = FINIT_WAIT_MEAS_PWMMAX;
									ui16CheckStatesDelay = 50;
								}
								else
								{
									ui16CheckStatesDelay--;
								}
							}
						}
					}
					break;
				}
				case FINIT_WAIT_MEAS_PWMMAX:
				{
					if(0 == FCFlightData->MOTORS.FR.ui8MeasuringPWMMax)
					{
						if(0 == FCFlightData->MOTORS.FL.ui8MeasuringPWMMax)
						{
							if(0 == FCFlightData->MOTORS.R.ui8MeasuringPWMMax)
							{
								// Set motor PWMs to min
								RCValues->ch[RC_MOTOR_FL].PWMOUT = 1000;
								RCValues->ch[RC_MOTOR_FR].PWMOUT = 1000;
								RCValues->ch[RC_MOTOR_R].PWMOUT = 1000;
								// Send command enable servo torque
								FCFlightData->TILT_SERVOS.FR.ui8Enable = 1;
								FCFlightData->TILT_SERVOS.FL.ui8Enable = 1;
								FCFlightData->TILT_SERVOS.R.ui8Enable = 1;

								FCFlightData->ui32FlightInitState = FINIT_WAIT_STORQUE_ON;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_STORQUE_ON:
				{
					// Check readout values
					if(1 == FCFlightData->TILT_SERVOS.FR.ui8Enabled)
					{
						if(1 == FCFlightData->TILT_SERVOS.FR.ui8Enabled)
						{
							if(1 == FCFlightData->TILT_SERVOS.FR.ui8Enabled)
							{
								// Wait for park position
								FCFlightData->ui32FlightInitState = FINIT_WAIT_MPOS_PARK;
							}
						}
					}
					break;
				}

				case FINIT_WAIT_MPOS_PARK:
				{
					//i16Temp = FCFlightData->MOTORS.FR.ui8Parked
					if(1 == FCFlightData->MOTORS.FR.ui8ParkPosition)
					{
						if(1 == FCFlightData->MOTORS.FL.ui8ParkPosition)
						{
							if(1 == FCFlightData->MOTORS.R.ui8ParkPosition)
							{
								// Set new position
								FCFlightData->f32NacelleTilt_FR = 0.0f;
								FCFlightData->f32NacelleTilt_FL = 0.0f;
								FCFlightData->f32NacelleTilt_R = 0.0f;
								// Wait
								FCFlightData->ui32FlightInitState = FINIT_WAIT_SPOS_LEVEL;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_SPOS_LEVEL:
				{
					if((ANGLE_DEV_LEVEL_N < FCFlightData->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_LEVEL_N < FCFlightData->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_LEVEL_N < FCFlightData->TILT_SERVOS.R.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData->TILT_SERVOS.R.f32ServoAngle))
							{
								// Motors ON


								// Set new position
								FCFlightData->f32NacelleTilt_FR = 90.0f;
								FCFlightData->f32NacelleTilt_FL = 90.0f;
								FCFlightData->f32NacelleTilt_R = 90.0f;
								// Wait
								FCFlightData->ui32FlightInitState = FINIT_WAIT_SPOS_VTOL;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_SPOS_VTOL:
				{
					if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.R.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.R.f32ServoAngle))
							{
								// Go out of park and enable PWM in
								// End init
								FCFlightData->ui32FlightInitState = FINIT_IDLE;
								FCFlightData->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
								RC_Flags.bits.HOVER = 1;
								RC_Flags.bits.PLANE = 0;
							}
						}
					}
					break;
				}
				default:
				{
					FCFlightData->ui32FlightInitState = FINIT_IDLE;
					break;
				}
			}
			break;
		}
		case FLIGHT_STABILIZE_HOVER:
		{
			// Run stabilising algorithm
			flight_stabilize(FCFlightData);
			break;
		}
		case FLIGHT_STABILIZE_PLANE:
		{
			// Run stabilising algorithm
			flight_stabilize(FCFlightData);
			break;
		}
		case FLIGHT_STABILIZE_TRANSITION:
		{
			// Run stabilising algorithm
			flight_stabilize(FCFlightData);

			// Tilt state machine
			switch(FCFlightData->ui32FlightTransitionState)
			{
				case FLIGHT_TILT_START:
				{
					// Still in transition?
					if(RC_Flags.bits.TRANSITION)
					{
						// Flight mode?
						if(RC_Flags.bits.PLANE)
						{
							// Tilt motors up
							FCFlightData->f32NacelleTilt_FR = 90.0f;
							FCFlightData->f32NacelleTilt_FL = 90.0f;
							FCFlightData->f32NacelleTilt_R = 90.0f;
							FCFlightData->ui32FlightTransitionState = FLIGHT_TILT_H;
						}
						else if(RC_Flags.bits.HOVER)
						{

						}
					}
					break;
				}
				case FLIGHT_TILT_P:
				{

					break;
				}
				case FLIGHT_TILT_H:
				{
					// Wait for tilt angle to be ~90 deg
					if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_VTOL_N < FCFlightData->TILT_SERVOS.R.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData->TILT_SERVOS.R.f32ServoAngle))
							{
								// Go to hover mode
								RC_Flags.bits.HOVER = 1;
								RC_Flags.bits.PLANE = 0;
								RC_Flags.bits.TRANSITION = 0;
								FCFlightData->ui32FlightTransitionState = FLIGHT_TILT_START;
							}
						}
					}
					break;
				}
				case FLIGHT_TILT_TILTED_P:
				{

					break;
				}
				case FLIGHT_TILT_TILTED_H:
				{

					break;
				}
				case FLIGHT_TILT_SPEED_OK_P:
				{

					break;
				}
				case FLIGHT_TILT_SPEED_OK_H:
				{
					break;
				}
				case FLIGHT_TILT_END_P:
				{
					break;
				}
				case FLIGHT_TILT_END_H:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_TILT_P:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_TILT_H:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_TILTED_P:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_TILTED_H:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_SPEED_OK_P:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_SPEED_OK_H:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_END_P:
				{
					break;
				}
				case FLIGHT_TILT_ABORT_END_H:
				{
					/*
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
					RC_Flags.bits.HOVER = 0;*/
					break;
				}
				default:
				{
					FCFlightData->ui32FlightTransitionState = FLIGHT_TILT_START;
				}
			}
			break;
		}
		case FLIGHT_DISARM:
		{
			// Send command disable servo torque
			FCFlightData->TILT_SERVOS.FR.ui8Enable = 0;
			FCFlightData->TILT_SERVOS.FL.ui8Enable = 0;
			FCFlightData->TILT_SERVOS.R.ui8Enable = 0;
			// Disable motors
			FCFlightData->MOTORS.FR.ui8Enable = 0;
			FCFlightData->MOTORS.FR.ui8Park = 0;
			FCFlightData->MOTORS.FL.ui8Enable = 0;
			FCFlightData->MOTORS.FL.ui8Park = 0;
			FCFlightData->MOTORS.R.ui8Enable = 0;
			FCFlightData->MOTORS.R.ui8Park = 0;
			FCFlightData->ui32FlightStateMachine = FLIGHT_IDLE;
			break;
		}

		default:
		{
			FCFlightData->ui32FlightStateMachine = FLIGHT_IDLE;
			break;
		}
	}
}

// Stabilize according to data
void flight_stabilize(FLIGHT_CORE * FCFlightData)
{
	float32_t f32Error = 0.0f;
	// Use PID regulator to stabilize vehicle
	// Stabilize roll - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED.f32Roll - FCFlightData->ORIENTATION.f32Roll;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDRoll);

	// Stabilize pitch - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED.f32Pitch - FCFlightData->ORIENTATION.f32Pitch;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDPitch);

	// Stabilize yaw - calculate error
	f32Error = FCFlightData->ORIENTATION_REQUIRED.f32Yaw - FCFlightData->ORIENTATION.f32Yaw;
	// Run PID
	math_PID(f32Error, 0.01f, &FCFlightData->PIDYaw);

	// Run altitude PID
	f32Error = FCFlightData->ORIENTATION_REQUIRED.f32Altitude - FCFlightData->ORIENTATION.f32Altitude;
	math_PID(f32Error, 0.01f, &FCFlightData->PIDAltitude);

	// Run speed PID

}

// Hardware specific function - decode flight commands to servo commands
void flight_decodeServos(FLIGHT_CORE * FCFlightData, RCDATA * RCValues)
{
	float32_t f32Power;
	float32_t f32Temp;
	float32_t f32Temp1;
	int i = 0;
	// Next depends on current mode
	if(RC_Flags.bits.PLANE)
	{

	}
	else if(RC_Flags.bits.HOVER)
	{
		// We are in hover mode, all maneuvering is done with motors, flaps/ailerons to middle
		RCValues->ch[RC_AILERON_L].PWMOUT = RCValues->ch[RC_AILERON_L].PWMIN_MID;
		RCValues->ch[RC_AILERON_R].PWMOUT = RCValues->ch[RC_AILERON_L].PWMIN_MID;

		// Power is altitude output
		f32Power = FCFlightData->PIDAltitude.s;

		// Calculate motor power ratios
		// Motor FR
		f32Temp = 0.5f-FCFlightData->PIDPitch.s;
		f32Temp1 = 0.5f-FCFlightData->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= f32Power;
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Store
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = f32Temp;

		// Motor FL
		f32Temp = 0.5f-FCFlightData->PIDPitch.s;
		f32Temp1 = 0.5f+FCFlightData->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= f32Power;
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Store
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = f32Temp;

		// Motor R
		f32Temp = 0.5f+FCFlightData->PIDPitch.s;
		f32Temp1 = cosf(FCFlightData->PIDYaw.s);
		f32Temp /= f32Temp1;
		// Get power
		f32Temp *= f32Power;
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Store
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = f32Temp;

		// Control yaw with nacelle roll
		// Let's assume that 1000 is 90 deg
		// So 1 deg is 11.11 q
		f32Temp = FCFlightData->PIDYaw.s * 11.11f;
		// Limit nacelle roll
		if(MAX_NACELLE_ROLL < f32Temp) f32Temp = MAX_NACELLE_ROLL;
		if(-MAX_NACELLE_ROLL > f32Temp) f32Temp = -MAX_NACELLE_ROLL;
		// Add zero PWM
		f32Temp += 1500;
		RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = f32Temp;

	}
	else
	{

	}

	//***********************************
	// Set tilt values
	// Set new position
	f32Temp = FCFlightData->f32NacelleTilt_FR + FCFlightData->TILT_SERVOS.FR.f32ServoZero;
	f32Temp *= 11.37777777777777;
	FCFlightData->TILT_SERVOS.FR.ui16RequestedPosition = (uint16_t)f32Temp;

	f32Temp = FCFlightData->f32NacelleTilt_FL + FCFlightData->TILT_SERVOS.FL.f32ServoZero;
	f32Temp *= 11.37777777777777;
	FCFlightData->TILT_SERVOS.FL.ui16RequestedPosition = (uint16_t)f32Temp;

	f32Temp = FCFlightData->f32NacelleTilt_R + FCFlightData->TILT_SERVOS.R.f32ServoZero;
	f32Temp *= 11.37777777777777;
	FCFlightData->TILT_SERVOS.R.ui16RequestedPosition = (uint16_t)f32Temp;
	//***********************************

	//***********************************
	// Set gear
	if(0 < RCValues->ch[RC_GEAR].PWMIN_MID)
	{
		// Gear up
		 RCValues->ch[RC_GEAR].PWMOUT = 1000;
	}
	else
	{
		// Gear down
		RCValues->ch[RC_GEAR].PWMOUT = 2000;
	}
	//***********************************

	//***********************************
	// Check arm
	if(0 < RCValues->ch[RC_FLAPS_PITCH].PWMIN_MID)
	{
		RC_Flags.bits.ARMED = 1;
		if(FLIGHT_IDLE == FCFlightData->ui32FlightStateMachine)
		{
			FCFlightData->ui32FlightStateMachine = FLIGHT_INIT;
			FCFlightData->ui32FlightInitState = FINIT_IDLE;
		}
	}
	else
	{
		RC_Flags.bits.ARMED = 0;
		if(FLIGHT_IDLE != FCFlightData->ui32FlightStateMachine)
		{
			if(FLIGHT_DISARM != FCFlightData->ui32FlightStateMachine)
			{
				FCFlightData->ui32FlightStateMachine = FLIGHT_DISARM;
				FCFlightData->ui32FlightInitState = FINIT_IDLE;
				FCFlightData->MOTORS.FR.i16PWMMin = 1000;
				FCFlightData->MOTORS.FL.i16PWMMin = 1000;
				FCFlightData->MOTORS.R.i16PWMMin = 1000;
				FCFlightData->MOTORS.FR.i16PWMMax = 2000;
				FCFlightData->MOTORS.FL.i16PWMMax = 2000;
				FCFlightData->MOTORS.R.i16PWMMax = 2000;
			}
		}
	}
	//***********************************

	//***********************************
	// Store pwm out values
	for(i=0; i < 12; i++)
	{
		RCValues->ch[i].PWMOUT = (uint16_t)RCValues->ch[i].PWMOUT_Val;
	}

	//***********************************
}


