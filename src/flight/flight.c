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
#define ANGLE_DEV_LEVEL_P		5.0f
#define ANGLE_DEV_LEVEL_N		-5.0f

#define ANGLE_DEV_VTOL_P		104.0f
#define ANGLE_DEV_VTOL_N		90.0f

// Max nacelle roll
#define MAX_NACELLE_ROLL		150.0f



// Init flight variables
void flight_init(FLIGHT_CORE *FCFlightData_local, RCDATA * RCValues)
{
	int i = 0;
	// Set initial state
	FCFlightData_local->ui32FlightStateMachine = FLIGHT_IDLE;

	FCFlightData_local->ui32FlightInitState = FINIT_IDLE;

	FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;

	// Altitude filter window
	FCFlightData_local->ORIENTATION.ui32AltitudeFilterWindow = 5.0f;

	// Init offsets
	FCFlightData_local->ORIENTATION.f32RollOffset = -0.05f;
	FCFlightData_local->ORIENTATION.f32PitchOffset = 0.0323f;
	FCFlightData_local->ORIENTATION.f32YawOffset = 0.0f;

	// Set COG distribution
	FCFlightData_local->f32COGFrontFactor = COG_BALANCE_FRONT;
	FCFlightData_local->f32COGRearFactor = COG_BALANCE_REAR;

	// Init PIDs
	math_PIDInit(&FCFlightData_local->PIDPitch, 0.20f, 0.01f, 0.10f, -0.15f, 0.15f);
	math_PIDInit(&FCFlightData_local->PIDRoll, 0.20f, 0.01f, 0.10f, -0.15f, 0.15f);
	math_PIDInit(&FCFlightData_local->PIDYaw, 2.0f, 0.0f, 0.0f, -0.06f, 0.06f);
	math_PIDInit(&FCFlightData_local->PIDAltitude, 0.1f, 0.01f, 0.0f, -0.10f, 0.10f);
	math_PIDInit(&FCFlightData_local->PIDSpeed, 0.1f, 0.01f, 0.0f, 0.15f, 1.0f);

	FCFlightData_local->f32COGDistribution = 0.6f;

	FCFlightData_local->ORIENTATION_REQUIRED.f32Roll = 0.0f;
	FCFlightData_local->ORIENTATION_REQUIRED.f32Pitch = 0.0f;
	FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = 0.0f;
	FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart = 0.0f;
	FCFlightData_local->ORIENTATION_REQUIRED.f32Speed = 0.0f;
	FCFlightData_local->ORIENTATION_REQUIRED.f32Power = 0.0f;

	// Limits in radians
	FCFlightData_local->ORIENTATION_LIMITS.f32RollLimit = 20.0f / 57.2957795f;
	FCFlightData_local->ORIENTATION_LIMITS.f32PitchLimit = 20.0f / 57.2957795f;
	FCFlightData_local->ORIENTATION_LIMITS.f32YawLimit = 360.0f / 57.2957795f;

	// Disable motors
	FCFlightData_local->MOTORS.FR.ui8Enable = 0;
	FCFlightData_local->MOTORS.FR.ui8Park = 0;
	FCFlightData_local->MOTORS.FR.ui8UsePWM = 0;
	FCFlightData_local->MOTORS.FL.ui8Enable = 0;
	FCFlightData_local->MOTORS.FL.ui8Park = 0;
	FCFlightData_local->MOTORS.FL.ui8UsePWM = 0;
	FCFlightData_local->MOTORS.RR.ui8Enable = 0;
	FCFlightData_local->MOTORS.RR.ui8Park = 0;
	FCFlightData_local->MOTORS.RR.ui8UsePWM = 0;
	FCFlightData_local->MOTORS.RL.ui8Enable = 0;
	FCFlightData_local->MOTORS.RL.ui8Park = 0;
	FCFlightData_local->MOTORS.RL.ui8UsePWM = 0;

	// Motor min/max RPM
	FCFlightData_local->MOTORS.FR.i16MinRPM = 2000;
	FCFlightData_local->MOTORS.FR.i16MaxRPM = 20000;
	FCFlightData_local->MOTORS.FL.i16MinRPM = 2000;
	FCFlightData_local->MOTORS.FL.i16MaxRPM = 20000;
	FCFlightData_local->MOTORS.RR.i16MinRPM = 2000;
	FCFlightData_local->MOTORS.RR.i16MaxRPM = 20000;
	FCFlightData_local->MOTORS.RL.i16MinRPM = 2000;
	FCFlightData_local->MOTORS.RL.i16MaxRPM = 20000;

	// Set motor park positions
	FCFlightData_local->MOTORS.FR.i16SetParkPosition = 2330;
	FCFlightData_local->MOTORS.FL.i16SetParkPosition = 2402;
	FCFlightData_local->MOTORS.RR.i16SetParkPosition = 1998;
	FCFlightData_local->MOTORS.RL.i16SetParkPosition = 1998;

	FCFlightData_local->f32MinPlaneSpeed = RC_DEFAULT_PLANE_MIN_SPEED;
	FCFlightData_local->f32MaxHoverSpeed = RC_DEFAULT_HOVER_MAX_SPEED;
	FCFlightData_local->f32NacelleTiltSpeed = RC_DEFAULT_TILT_SPEED;
	FCFlightData_local->f32NacellePlaneTransitionTilt = RC_NACELLE_TRANSITION_TILT;
	FCFlightData_local->f32NacelleHoverTransitionTilt = RC_NACELLE_HOVERTRANSITION_TILT;

	// Set nacelle tilt state
	FCFlightData_local->f32NacelleCommonTilt = RC_DEFAULT_NACELLE_TILT;
	FCFlightData_local->f32NacelleTilt_FL = RC_DEFAULT_NACELLE_TILT;
	FCFlightData_local->f32NacelleTilt_FR = RC_DEFAULT_NACELLE_TILT;
	FCFlightData_local->f32NacelleTilt_RR = RC_DEFAULT_NACELLE_TILT;
	// Set zero angle values
	FCFlightData_local->TILT_SERVOS.f32AllowedPositionDeviation = 20.0f;	// ~2 deg
	FCFlightData_local->TILT_SERVOS.FR.f32ServoZero = NACELLE_FR_ZERO;
	FCFlightData_local->TILT_SERVOS.FL.f32ServoZero = NACELLE_FL_ZERO;
	FCFlightData_local->TILT_SERVOS.RR.f32ServoZero = NACELLE_R_ZERO;
	FCFlightData_local->TILT_SERVOS.FR.ui8Reverse = 1;
	FCFlightData_local->TILT_SERVOS.FL.ui8Reverse = 0;
	FCFlightData_local->TILT_SERVOS.RR.ui8Reverse = 0;

	RCValues->SCALES.f32ElevatorScale = 0.1;
	RCValues->SCALES.f32AileronScale = 0.1;

	RCValues->inputs_ok = 0;

	for(i=0; i < 12; i++)
	{
		RCValues->ch[i].PWMIN_MID = RC_IN_DEFAULT_MIDPOINT;
		RCValues->ch[i].PWMOUT_Offset = RC_IN_DEFAULT_MIDPOINT;
		RCValues->ch[i].PWMIN_DeadZone = RC_IN_DEADZONE;

		RCValues->ch[i].PWMMin = 1200;
		RCValues->ch[i].PWMMax = 1800;
		RCValues->ch[i].PWMDiff = 600;
		RCValues->ch[i].PWMOUT = 1500;
		RCValues->ch[i].PWMOUT_Val = 1500;
		RCValues->ch[i].PWM_Good = 0;
		RCValues->ch[i].PWM_Timeout = 0;
	}
	// Rudder has larger deadzone.
	RCValues->ch[RC_RUDDER].PWMIN_DeadZone = 25.0f;


	// Set default PWM outputs

	// Gear down
	RCValues->ch[RC_GEAR].PWMOUT_Val = 1000;
	RCValues->ch[RC_GEAR].PWMOUT = 1000;

	// Ailerons to midpoint
	RCValues->ch[RC_AILERON_L].PWMOUT = 1500;
	RCValues->ch[RC_AILERON_L].PWMOUT_Val = 1500;
	RCValues->ch[RC_AILERON_R].PWMOUT = 1500;
	RCValues->ch[RC_AILERON_R].PWMOUT_Val = 1500;
	// Motors to zero
	RCValues->ch[RC_MOTOR_FL].PWMOUT = 1000;
	RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
	RCValues->ch[RC_MOTOR_FR].PWMOUT = 1000;
	RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
	RCValues->ch[RC_MOTOR_RR].PWMOUT = 1000;
	RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = 1000;


	// Nacelle tilt to midpoint
	RCValues->ch[RC_MOTOR_R_TILT].PWMOUT = RC_OUT_YAW_MIDPOINT;
	RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RC_OUT_YAW_MIDPOINT;
	RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Offset = RC_OUT_YAW_MIDPOINT;
	RCValues->i16YawValue = RC_OUT_YAW_MIDPOINT;
	// Mark run yaw PID
	FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID = 1;


	RCValues->SCALES.f32AileronScale =  FCFlightData_local->ORIENTATION_LIMITS.f32RollLimit / RCValues->ch[RC_AILERON].PWMDiff;
	RCValues->SCALES.f32ElevatorScale = FCFlightData_local->ORIENTATION_LIMITS.f32PitchLimit / RCValues->ch[RC_ELEVATOR].PWMDiff;

	RCValues->SCALES.f32RudderScale = RC_IN_DEFAULT_SCALE_RUDDER;
	RCValues->SCALES.f32ThrottleScale = RC_IN_DEFAULT_SCALE_THROTTLE;
	// Reset mAh counter
	FCFlightData_local->batMon.fmAhUsed = 0.0f;
	FCFlightData_local->batMon.ui16NumCells = 4;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[0] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[1] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[2] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[3] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[4] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[5] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[6] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[7] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[8] = 65535;
	FCFlightData_local->batMon.ui16MavlinkBatteryVoltages[9] = 65535;
}

// What to do if there is no RC input
int16_t flight_decideAction(FLIGHT_CORE * FCFlightData_local, RCDATA * RCValues)
{
	// Gear down
	RCValues->ch[RC_GEAR].PWMOUT = 1000;

	// Disarm
	RC_Flags.bits.ARMED = 0;
	if(FLIGHT_IDLE != FCFlightData_local->ui32FlightStateMachine)
	{
		if(FLIGHT_DISARM != FCFlightData_local->ui32FlightStateMachine)
		{
			FCFlightData_local->ui32FlightStateMachine = FLIGHT_DISARM;
			FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
		}
	}
	return 0;
}

// Decode what RC commands want
void flight_checkRCInputs(FLIGHT_CORE * FCFlightData_local, RCDATA * RCValues)
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
		FCFlightData_local->ORIENTATION.f32ZeroAltitude = FCFlightData_local->ORIENTATION.f32Altitude;
		// Store current heading as zero heading
		FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = FCFlightData_local->ORIENTATION.f32Yaw;
	}

	// Check input limits
	for(i=0; i < 8; i++)
	{
		f32Temp = (float32_t)RCValues->ch[i].PWMIN;
		// Over max, under min
		// PWM value makes sense?
		if((2300 > RCValues->ch[i].PWMIN)&&(700 < RCValues->ch[i].PWMIN))
		{
			if(f32Temp < RCValues->ch[i].PWMMin)
			{
				RCValues->ch[i].PWMMin = f32Temp;
				RCValues->ch[i].PWMDiff = RCValues->ch[i].PWMMax - RCValues->ch[i].PWMMin;
				// Recalculate scales
				if(RC_AILERON == i)
				{
					RCValues->SCALES.f32AileronScale =  FCFlightData_local->ORIENTATION_LIMITS.f32RollLimit / RCValues->ch[RC_AILERON].PWMDiff;
				}
				else if(RC_ELEVATOR == i)
				{
					RCValues->SCALES.f32ElevatorScale = FCFlightData_local->ORIENTATION_LIMITS.f32PitchLimit / RCValues->ch[RC_ELEVATOR].PWMDiff;
				}
			}
			if(f32Temp > RCValues->ch[i].PWMMax)
			{
				RCValues->ch[i].PWMMax = f32Temp;
				RCValues->ch[i].PWMDiff = RCValues->ch[i].PWMMax - RCValues->ch[i].PWMMin;
				// Recalculate scales
				if(RC_AILERON == i)
				{
					RCValues->SCALES.f32AileronScale = FCFlightData_local->ORIENTATION_LIMITS.f32RollLimit / RCValues->ch[RC_AILERON].PWMDiff;
				}
				else if(RC_ELEVATOR == i)
				{
					RCValues->SCALES.f32ElevatorScale = FCFlightData_local->ORIENTATION_LIMITS.f32PitchLimit / RCValues->ch[RC_ELEVATOR].PWMDiff;
				}
			}
			// Calculate current value
			RCValues->ch[i].PWMIN_Zero = (float32_t)RCValues->ch[i].PWMIN - RCValues->ch[i].PWMIN_MID;
			// Remove deadzone
			if((RCValues->ch[i].PWMIN_DeadZone > RCValues->ch[i].PWMIN_Zero) && (-RCValues->ch[i].PWMIN_DeadZone < RCValues->ch[i].PWMIN_Zero))
			{
				RCValues->ch[i].PWMIN_Zero = 0.0f;
			}
		}
	}
	// Decode RC inputs and turn them into flight data commands

	// If hover
	if(RC_Flags.bits.HOVER)
	{
		// Turn throttle to limit for altitude reg.
		f32Temp = (float32_t)RCValues->ch[RC_THROTTLE].PWMIN;
		f32Temp -= RCValues->ch[RC_THROTTLE].PWMMin;
		f32Temp /= RCValues->ch[RC_THROTTLE].PWMDiff;
		f32Temp = 1 - f32Temp;
		f32Temp = f32Temp * 1.5;
		f32Temp -= 0.05f;
		if(0.0f > f32Temp) f32Temp = 0.0f;

		RCValues->f32ThrottleValue = f32Temp;

		/*
		FCFlightData_local->PIDAltitude.outMax = f32Temp;//(RCValues->ch[RC_THROTTLE].PWMIN_Zero / RCValues->ch[RC_THROTTLE].PWMDiff) * 2.0f;

		// And use it for altitude adjust

		FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart += 0.0001f*(RCValues->ch[RC_THROTTLE].PWMIN_Zero);
		// Limit altitude
		if(400.0f < FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart)
		{
			FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart = 400.0f;
		}
		else if(0.0f > FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart)
		{
			FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart = 0.0f;
		}
		*/

		// Elevator controls pitch
		f32Temp = RCValues->ch[RC_ELEVATOR].PWMIN_Zero * RCValues->SCALES.f32ElevatorScale;
		FCFlightData_local->ORIENTATION_REQUIRED.f32Pitch = f32Temp;
		// Aileron controls roll
		f32Temp = RCValues->ch[RC_AILERON].PWMIN_Zero * RCValues->SCALES.f32AileronScale;
		FCFlightData_local->ORIENTATION_REQUIRED.f32Roll = f32Temp;


		// Rudder controls yaw
		f32Temp = RCValues->ch[RC_RUDDER].PWMIN_Zero  * RCValues->SCALES.f32RudderScale;

		if((5.0f > f32Temp)&&(-5.0f < f32Temp))
		{
			FCFlightData_local->f32YawCommand = 0;
		}
		else
		{
			FCFlightData_local->f32YawCommand = f32Temp;
		}
		/*
		// Add to required yaw
		FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw += f32Temp;
		// Limit yaw to +/- 180
		if(3.141593f < FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw)
		{
			FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = -3.141593f;
		}
		else if(-3.141593f > FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw)
		{
			FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = 3.141593f;
		}
		*/

	}
	else
	{
		// Turn throttle to limit for altitude reg.
		f32Temp = (float32_t)RCValues->ch[RC_THROTTLE].PWMIN;
		f32Temp -= RCValues->ch[RC_THROTTLE].PWMMin;
		f32Temp /= RCValues->ch[RC_THROTTLE].PWMDiff;
		f32Temp = 1 - f32Temp;
		f32Temp -= 0.05f;
		if(0.0f > f32Temp) f32Temp = 0.0f;
		RCValues->f32ThrottleValue = f32Temp;
	}
	//***********************************
	// Set gear
	if(0 < RCValues->ch[RC_GEAR_GYRO].PWMIN_Zero)
	{
		// Gear up
		RCValues->ch[RC_GEAR].PWMOUT_Val = 2000;
		/*
		if((1 == FCFlightData_local->MOTORS.FL.ui8Enable)&&
				(1 == FCFlightData_local->MOTORS.FR.ui8Enable)&&
				(1 == FCFlightData_local->MOTORS.RL.ui8Enable)&&
				(1 == FCFlightData_local->MOTORS.RR.ui8Enable))
		{
			enableMotors(1);
		}*/
	}
	else
	{
		// Gear down
		 RCValues->ch[RC_GEAR].PWMOUT_Val = 1000;

		 //enableMotors(0);
	}
	//***********************************

	//***********************************
	// Check arm
	if(0 < RCValues->ch[RC_FLAPS_PITCH].PWMIN_Zero)
	{
		RC_Flags.bits.ARMED = 1;
		if(FLIGHT_IDLE == FCFlightData_local->ui32FlightStateMachine)
		{
			FCFlightData_local->ui32FlightStateMachine = FLIGHT_INIT;
			FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
			FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
		}
	}
	else
	{
		RC_Flags.bits.ARMED = 0;
		if(FLIGHT_IDLE != FCFlightData_local->ui32FlightStateMachine)
		{
			if(FLIGHT_DISARM != FCFlightData_local->ui32FlightStateMachine)
			{
				FCFlightData_local->ui32FlightStateMachine = FLIGHT_DISARM;
				FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
				FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
			}
		}
	}
	//***********************************
}

// Main flight state machine
void flight_checkStatesY(FLIGHT_CORE *FCFlightData_local, RCDATA * RCValues)
{
	//int i = 0;
	switch(FCFlightData_local->ui32FlightStateMachine)
	{
		case FLIGHT_IDLE:
		{
			break;
		}
		case FLIGHT_INIT:
		{
			switch(FCFlightData_local->ui32FlightInitState)
			{
				case FINIT_IDLE:
				{
					// Set motor PWMs to min
					RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
					RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
					RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = 1000;
					// Reverse motor FR
					FCFlightData_local->MOTORS.FR.ui8ReverseRotation = 0;
					FCFlightData_local->MOTORS.FL.ui8ReverseRotation = 1;
					FCFlightData_local->MOTORS.RR.ui8ReverseRotation = 0;
					// Enable motors
					FCFlightData_local->MOTORS.FR.ui8Enable = 1;
					FCFlightData_local->MOTORS.FL.ui8Enable = 1;
					FCFlightData_local->MOTORS.RR.ui8Enable = 1;
					// Set to park
					FCFlightData_local->MOTORS.FR.ui8Park = 1;
					FCFlightData_local->MOTORS.FL.ui8Park = 1;
					FCFlightData_local->MOTORS.RR.ui8Park = 1;
					// Mark measure low PWM time
					FCFlightData_local->MOTORS.FR.ui8MeasPWMMin = 1;
					FCFlightData_local->MOTORS.FL.ui8MeasPWMMin = 1;
					FCFlightData_local->MOTORS.RR.ui8MeasPWMMin = 1;
					// Next state
					FCFlightData_local->ui32FlightInitState = FINIT_MEAS_PWM_MIN;
					ui16CheckStatesDelay = 50;
					break;
				}
				case FINIT_MEAS_PWM_MIN:
				{
					// Check if values are updated
					if(1 == FCFlightData_local->MOTORS.FR.ui8MeasuringPWMMin)
					{
						if(1 == FCFlightData_local->MOTORS.FL.ui8MeasuringPWMMin)
						{
							if(1 == FCFlightData_local->MOTORS.RR.ui8MeasuringPWMMin)
							{
								if(0 == ui16CheckStatesDelay)
								{
									// Mark measure low PWM time
									FCFlightData_local->MOTORS.FR.ui8MeasPWMMin = 0;
									FCFlightData_local->MOTORS.FL.ui8MeasPWMMin = 0;
									FCFlightData_local->MOTORS.RR.ui8MeasPWMMin = 0;

									// Set servo max torque
									FCFlightData_local->TILT_SERVOS.FR.ui16RequestedMaxTorque = 6000;
									FCFlightData_local->TILT_SERVOS.FL.ui16RequestedMaxTorque = 6000;
									FCFlightData_local->TILT_SERVOS.RR.ui16RequestedMaxTorque = 6000;

									// Next state
									FCFlightData_local->ui32FlightInitState = FINIT_WAIT_MEAS_PWMMIN;
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
					if(0 == FCFlightData_local->MOTORS.FR.ui8MeasuringPWMMin)
					{
						if(0 == FCFlightData_local->MOTORS.FL.ui8MeasuringPWMMin)
						{
							if(0 == FCFlightData_local->MOTORS.RR.ui8MeasuringPWMMin)
							{
								FCFlightData_local->MOTORS.FR.ui8MeasPWMMax = 1;
								FCFlightData_local->MOTORS.FL.ui8MeasPWMMax = 1;
								FCFlightData_local->MOTORS.RR.ui8MeasPWMMax = 1;
								// Set motor PWMs to max
								RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 2000;
								RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 2000;
								RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = 2000;
								//RCValues->ch[RC_MOTOR_RL].PWMOUT_Val = 2000;
								FCFlightData_local->ui32FlightInitState = FINIT_MEAS_PWM_MAX;
							}
						}
					}
					break;
				}
				case FINIT_MEAS_PWM_MAX:
				{
					if(1 == FCFlightData_local->MOTORS.FR.ui8MeasuringPWMMax)
					{
						if(1 == FCFlightData_local->MOTORS.FL.ui8MeasuringPWMMax)
						{
							if(1 == FCFlightData_local->MOTORS.RR.ui8MeasuringPWMMax)
							{
								if(0 == ui16CheckStatesDelay)
								{
									FCFlightData_local->MOTORS.FR.ui8MeasPWMMax = 0;
									FCFlightData_local->MOTORS.FL.ui8MeasPWMMax = 0;
									FCFlightData_local->MOTORS.RR.ui8MeasPWMMax = 0;
									// Next state
									FCFlightData_local->ui32FlightInitState = FINIT_WAIT_MEAS_PWMMAX;
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
					if(0 == FCFlightData_local->MOTORS.FR.ui8MeasuringPWMMax)
					{
						if(0 == FCFlightData_local->MOTORS.FL.ui8MeasuringPWMMax)
						{
							if(0 == FCFlightData_local->MOTORS.RR.ui8MeasuringPWMMax)
							{
								// Set motor PWMs to min
								RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
								RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
								RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = 1000;
								// Send command enable servo torque
								FCFlightData_local->TILT_SERVOS.FR.ui8Enable = 1;
								FCFlightData_local->TILT_SERVOS.FL.ui8Enable = 1;
								FCFlightData_local->TILT_SERVOS.RR.ui8Enable = 1;

								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_STORQUE_ON;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_STORQUE_ON:
				{
					// Check readout values
					if(1 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
					{
						if(1 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
						{
							if(1 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
							{
								// Wait for park position
								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_MPOS_PARK;
							}
						}
					}
					break;
				}

				case FINIT_WAIT_MPOS_PARK:
				{
					//i16Temp = FCFlightData_local->MOTORS.FR.ui8Parked
					if(1 == FCFlightData_local->MOTORS.FR.ui8ParkPosition)
					{
						if(1 == FCFlightData_local->MOTORS.FL.ui8ParkPosition)
						{
							if(1 == FCFlightData_local->MOTORS.RR.ui8ParkPosition)
							{
								// Set new position
								FCFlightData_local->f32NacelleTilt_FR = 0.0f;
								FCFlightData_local->f32NacelleTilt_FL = 0.0f;
								FCFlightData_local->f32NacelleTilt_RR = 0.0f;
								// Wait
								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_SPOS_LEVEL;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_SPOS_LEVEL:
				{
					if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle))
							{
								// Set new position
								FCFlightData_local->f32NacelleTilt_FR = 94.0f;
								FCFlightData_local->f32NacelleTilt_FL = 94.0f;
								FCFlightData_local->f32NacelleTilt_RR = 97.0f;
								// Wait
								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_SPOS_VTOL;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_SPOS_VTOL:
				{
					if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle))
							{
								// Set servo max torque
								FCFlightData_local->TILT_SERVOS.FR.ui16RequestedMaxTorque = 2000;
								FCFlightData_local->TILT_SERVOS.FL.ui16RequestedMaxTorque = 2000;
								FCFlightData_local->TILT_SERVOS.RR.ui16RequestedMaxTorque = 2000;
								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_IMAX_SET;
							}
						}
					}
					break;
				}
				case FINIT_WAIT_IMAX_SET:
				{
					if(FCFlightData_local->TILT_SERVOS.FR.ui16MaxTorque == FCFlightData_local->TILT_SERVOS.FR.ui16RequestedMaxTorque)
					{
						if(FCFlightData_local->TILT_SERVOS.FL.ui16MaxTorque == FCFlightData_local->TILT_SERVOS.FL.ui16RequestedMaxTorque)
						{
							if(FCFlightData_local->TILT_SERVOS.RR.ui16MaxTorque == FCFlightData_local->TILT_SERVOS.RR.ui16RequestedMaxTorque)
							{
								// Wait throttle null
								FCFlightData_local->ui32FlightInitState = FINIT_WAIT_THROTTLE_NULL;

								// Set servos to holding position
								FCFlightData_local->f32NacelleTilt_FR = 95.0f;
								FCFlightData_local->f32NacelleTilt_FL = 100.0f;
								FCFlightData_local->f32NacelleTilt_RR = 100.0f;
							}
						}
					}

					break;
				}
				case FINIT_WAIT_THROTTLE_NULL:
				{
					if(0.05f > RCValues->f32ThrottleValue)
					{
						// Go out of park and enable PWM in
						// Disable PWM inputs for testinf
						FCFlightData_local->MOTORS.FR.ui8Park = 0;
						FCFlightData_local->MOTORS.FL.ui8Park = 0;
						FCFlightData_local->MOTORS.RR.ui8Park = 0;
						FCFlightData_local->MOTORS.FR.ui8UsePWM = 1;
						FCFlightData_local->MOTORS.FL.ui8UsePWM = 1;
						FCFlightData_local->MOTORS.RR.ui8UsePWM = 1;

						// End init, go to hover stabilize
						FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
						FCFlightData_local->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
						RC_Flags.bits.HOVER = 1;
						RC_Flags.bits.PLANE = 0;
						RC_Flags.bits.TRANSITION = 0;
					}
					break;
				}
				default:
				{
					FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
					break;
				}
			}
			break;
		}
		case FLIGHT_STABILIZE_HOVER:
		{
			// Check that we have throttle
			// -50
			if(-50.0f > RCValues->ch[RC_THROTTLE].PWMIN_Zero)
			{
				// Run stabilising algorithm
				flight_stabilize(FCFlightData_local);
			}
			else
			{
				// Else reset integrators in PID control
				FCFlightData_local->PIDRoll.im = 0.0f;
				FCFlightData_local->PIDPitch.im = 0.0f;
				FCFlightData_local->PIDYaw.im = 0.0f;
				FCFlightData_local->PIDAltitude.im = 0.0f;
				// Reset yaw and altitude
				// Store current altitude as zero altitude
				FCFlightData_local->ORIENTATION.f32ZeroAltitude = FCFlightData_local->ORIENTATION.f32Altitude;
				// Store current heading as zero heading
				FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = FCFlightData_local->ORIENTATION.f32Yaw;
			}

			break;
		}
		case FLIGHT_STABILIZE_PLANE:
		{
			// Run stabilising algorithm
			flight_stabilize(FCFlightData_local);
			break;
		}
		case FLIGHT_STABILIZE_TRANSITION:
		{
			// Run stabilising algorithm
			flight_stabilize(FCFlightData_local);

			// Tilt state machine
			switch(FCFlightData_local->ui32FlightTransitionState)
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
							FCFlightData_local->f32NacelleTilt_FR = 90.0f;
							FCFlightData_local->f32NacelleTilt_FL = 90.0f;
							FCFlightData_local->f32NacelleTilt_RR = 90.0f;
							FCFlightData_local->ui32FlightTransitionState = FLIGHT_TILT_H;
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
					if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_VTOL_N < FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle)&&(ANGLE_DEV_VTOL_P > FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle))
							{
								// Go to hover mode
								RC_Flags.bits.HOVER = 1;
								RC_Flags.bits.PLANE = 0;
								RC_Flags.bits.TRANSITION = 0;
								FCFlightData_local->ui32FlightTransitionState = FLIGHT_TILT_START;
							}
						}
					}
					break;
				}
				default:
				{
					FCFlightData_local->ui32FlightTransitionState = FLIGHT_TILT_START;
				}
			}
			break;
		}
		case FLIGHT_DISARM:
		{
			// Go out of modes
			RC_Flags.bits.HOVER = 0;
			RC_Flags.bits.PLANE = 0;

			// Gear down
			//RCValues->ch[RC_GEAR].PWMOUT_Val = 1000;
			// Ailerons to midpoint
			RCValues->ch[RC_AILERON_L].PWMOUT_Val = 1500;
			RCValues->ch[RC_AILERON_R].PWMOUT_Val = 1500;
			// Motors to zero
			RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
			RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
			RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = 1000;
			// Motor req speed to zero
			FCFlightData_local->MOTORS.FR.i16ReqRPM = 0;
			FCFlightData_local->MOTORS.FL.i16ReqRPM = 0;
			FCFlightData_local->MOTORS.RR.i16ReqRPM = 0;
			// Nacelle tilt to midpoint
			if(RC_OUT_YAW_MIDPOINT > RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
			{
				RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val += 1.0f;
			}
			else if(RC_OUT_YAW_MIDPOINT < RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
			{
				RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val -= 1.0f;
			}
			switch(FCFlightData_local->ui32FlightDeInitStates)
			{
				case FDEINIT_IDLE:
				{
					FCFlightData_local->MOTORS.FR.i16ReqRPM = 0;
					FCFlightData_local->MOTORS.FL.i16ReqRPM = 0;
					FCFlightData_local->MOTORS.RR.i16ReqRPM = 0;
					FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_MOTOR_STOP;
					break;
				}
				case FDEINIT_WAIT_MOTOR_STOP:
				{
					if(100 > FCFlightData_local->MOTORS.FR.i16CurrentRPM)
					{
						if(100 > FCFlightData_local->MOTORS.FL.i16CurrentRPM)
						{
							if(100 > FCFlightData_local->MOTORS.RR.i16CurrentRPM)
							{
								// Motors stopped
								// Motor req speed to zero
								FCFlightData_local->MOTORS.FR.i16ReqRPM = 0;
								FCFlightData_local->MOTORS.FL.i16ReqRPM = 0;
								FCFlightData_local->MOTORS.RR.i16ReqRPM = 0;
								// Disable PWM control
								FCFlightData_local->MOTORS.FR.ui8UsePWM = 0;
								FCFlightData_local->MOTORS.FL.ui8UsePWM = 0;
								FCFlightData_local->MOTORS.RR.ui8UsePWM = 0;
								// Go to park
								FCFlightData_local->MOTORS.FR.ui8Park = 1;
								FCFlightData_local->MOTORS.FL.ui8Park = 1;
								FCFlightData_local->MOTORS.RR.ui8Park = 1;
								// Enable motor for park
								FCFlightData_local->MOTORS.FR.ui8Enable = 1;
								FCFlightData_local->MOTORS.FL.ui8Enable = 1;
								FCFlightData_local->MOTORS.RR.ui8Enable = 1;

								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_MOTOR_PARK;
							}
						}
					}
					break;
				}
				case FDEINIT_WAIT_MOTOR_PARK:
				{
					if(1 == FCFlightData_local->MOTORS.FR.ui8ParkPosition)
					{
						if(1 == FCFlightData_local->MOTORS.FL.ui8ParkPosition)
						{
							if(1 == FCFlightData_local->MOTORS.RR.ui8ParkPosition)
							{
								// Wait PWM disabled
								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_PWM_DISABLED;
							}
						}
					}
					break;
				}
				case FDEINIT_WAIT_PWM_DISABLED:
				{
					if(0 == FCFlightData_local->MOTORS.FR.ui8UsingPWM)
					{
						if(0 == FCFlightData_local->MOTORS.FL.ui8UsingPWM)
						{
							if(0 == FCFlightData_local->MOTORS.RR.ui8UsingPWM)
							{
								// Disable motors
								FCFlightData_local->MOTORS.FR.ui8Enable = 0;
								FCFlightData_local->MOTORS.FL.ui8Enable = 0;
								FCFlightData_local->MOTORS.RR.ui8Enable = 0;
								// Disable park
								FCFlightData_local->MOTORS.FR.ui8Park = 0;
								FCFlightData_local->MOTORS.FL.ui8Park = 0;
								FCFlightData_local->MOTORS.RR.ui8Park = 0;

								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_MOTORS_DISABLED;
							}
						}
					}
					break;
				}
				case FDEINIT_WAIT_MOTORS_DISABLED:
				{
					if(0 == FCFlightData_local->MOTORS.FR.ui8Enabled)
					{
						if(0 == FCFlightData_local->MOTORS.FL.ui8Enabled)
						{
							if(0 == FCFlightData_local->MOTORS.RR.ui8Enabled)
							{
								// Tilt to plane
								FCFlightData_local->f32NacelleTilt_FR = 0.0f;
								FCFlightData_local->f32NacelleTilt_FL = 0.0f;
								FCFlightData_local->f32NacelleTilt_RR = 0.0f;

								// Set servo max torque
								FCFlightData_local->TILT_SERVOS.FR.ui16RequestedMaxTorque = 6000;
								FCFlightData_local->TILT_SERVOS.FL.ui16RequestedMaxTorque = 6000;
								FCFlightData_local->TILT_SERVOS.RR.ui16RequestedMaxTorque = 6000;

								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_TILT_PLANE;
							}
						}
					}
					break;
				}
				case FDEINIT_WAIT_TILT_PLANE:
				{
					if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.FR.f32ServoAngle))
					{
						if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.FL.f32ServoAngle))
						{
							if((ANGLE_DEV_LEVEL_N < FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle)&&(ANGLE_DEV_LEVEL_P > FCFlightData_local->TILT_SERVOS.RR.f32ServoAngle))
							{
								FCFlightData_local->TILT_SERVOS.FR.ui8Enable = 0;
								FCFlightData_local->TILT_SERVOS.FL.ui8Enable = 0;
								FCFlightData_local->TILT_SERVOS.RR.ui8Enable = 0;
								// Wait
								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_SERVO_DISABLED;
							}
						}
					}

					break;
				}
				case FDEINIT_WAIT_SERVO_DISABLED:
				{
					if(0 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
					{
						if(0 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
						{
							if(0 == FCFlightData_local->TILT_SERVOS.FR.ui8Enabled)
							{
								FCFlightData_local->ui32FlightStateMachine = FLIGHT_IDLE;
								// Go to idle
								FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
							}
						}
					}
					break;
				}
				default:
				{
					FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
					break;
				}
			}
			break;
		}
		default:
		{
			FCFlightData_local->ui32FlightStateMachine = FLIGHT_IDLE;
			FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
			FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
			break;
		}
	}
}
#define MOTOR_REQ_NULL_RPM		2000
// Main flight state machine
void flight_checkStatesQ(FLIGHT_CORE *FCFlightData_local, RCDATA * RCValues)
{
	//int i = 0;
	switch(FCFlightData_local->ui32FlightStateMachine)
	{
		case FLIGHT_IDLE:
		{
			break;
		}
		case FLIGHT_INIT:
		{
			switch(FCFlightData_local->ui32FlightInitState)
			{
				case FINIT_IDLE:
				{
					// Set RPM
					FCFlightData_local->MOTORS.FR.i16ReqRPM = MOTOR_REQ_NULL_RPM;
					FCFlightData_local->MOTORS.FL.i16ReqRPM = MOTOR_REQ_NULL_RPM;
					FCFlightData_local->MOTORS.RR.i16ReqRPM = MOTOR_REQ_NULL_RPM;
					FCFlightData_local->MOTORS.RL.i16ReqRPM = MOTOR_REQ_NULL_RPM;

					// Next state
					FCFlightData_local->ui32FlightInitState = FINIT_WAIT_RPM_SET;
					ui16CheckStatesDelay = 50;
					break;
				}
				case FINIT_WAIT_RPM_SET:
				{
					if(MOTOR_REQ_NULL_RPM == FCFlightData_local->MOTORS.FL.i16ReqRPM)
					{
						if(MOTOR_REQ_NULL_RPM == FCFlightData_local->MOTORS.FR.i16ReqRPM)
						{
							if(MOTOR_REQ_NULL_RPM == FCFlightData_local->MOTORS.RL.i16ReqRPM)
							{
								if(MOTOR_REQ_NULL_RPM == FCFlightData_local->MOTORS.RR.i16ReqRPM)
								{
									if(0 == ui16CheckStatesDelay)
									{
										if(0.05f > RCValues->f32ThrottleValue)
										{
											// Enable motors
											FCFlightData_local->MOTORS.FR.ui8Enable = 1;
											FCFlightData_local->MOTORS.FL.ui8Enable = 1;
											FCFlightData_local->MOTORS.RR.ui8Enable = 1;
											FCFlightData_local->MOTORS.RL.ui8Enable = 1;

											enableMotors(1);

											// Next state
											ui16CheckStatesDelay = 50;
											// End init, go to hover stabilize
											FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
											FCFlightData_local->ui32FlightStateMachine = FLIGHT_STABILIZE_HOVER;
											RC_Flags.bits.HOVER = 1;
											RC_Flags.bits.PLANE = 0;
											RC_Flags.bits.TRANSITION = 0;
										}
									}
									else
									{
										ui16CheckStatesDelay--;
									}
								}
							}
						}
					}
					break;
				}
				default:
				{
					FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
					break;
				}
			}
			break;
		}
		case FLIGHT_STABILIZE_HOVER:
		{
			// Check that we have throttle
			// -50
			if(-50.0f > RCValues->ch[RC_THROTTLE].PWMIN_Zero)
			{
				// Run stabilising algorithm
				flight_stabilize(FCFlightData_local);
			}
			else
			{
				// Else reset integrators in PID control
				FCFlightData_local->PIDRoll.im = 0.0f;
				FCFlightData_local->PIDPitch.im = 0.0f;
				FCFlightData_local->PIDYaw.im = 0.0f;
				FCFlightData_local->PIDAltitude.im = 0.0f;
				// Reset yaw and altitude
				// Store current altitude as zero altitude
				FCFlightData_local->ORIENTATION.f32ZeroAltitude = FCFlightData_local->ORIENTATION.f32Altitude;
				// Store current heading as zero heading
				FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = FCFlightData_local->ORIENTATION.f32Yaw;
			}

			break;
		}
		case FLIGHT_DISARM:
		{
			// Go out of modes
			RC_Flags.bits.HOVER = 0;
			RC_Flags.bits.PLANE = 0;

			// Gear down
			//RCValues->ch[RC_GEAR].PWMOUT_Val = 1000;
			// Ailerons to midpoint
			RCValues->ch[RC_AILERON_L].PWMOUT_Val = 1500;
			RCValues->ch[RC_AILERON_R].PWMOUT_Val = 1500;
			// Disable motors
			FCFlightData_local->MOTORS.FR.ui8Enable = 0;
			FCFlightData_local->MOTORS.FL.ui8Enable = 0;
			FCFlightData_local->MOTORS.RR.ui8Enable = 0;
			FCFlightData_local->MOTORS.RL.ui8Enable = 0;

			enableMotors(0);

			switch(FCFlightData_local->ui32FlightDeInitStates)
			{
				case FDEINIT_IDLE:
				{
					FCFlightData_local->ui32FlightDeInitStates = FDEINIT_WAIT_MOTORS_DISABLED;
					break;
				}
				case FDEINIT_WAIT_MOTORS_DISABLED:
				{
					if(0 != FCFlightData_local->MOTORS.FL.i16CurrentRPM)
					{
						if(0 != FCFlightData_local->MOTORS.FR.i16CurrentRPM)
						{
							if(0 != FCFlightData_local->MOTORS.RL.i16CurrentRPM)
							{
								if(0 != FCFlightData_local->MOTORS.RR.i16CurrentRPM)
								{
									FCFlightData_local->ui32FlightStateMachine = FLIGHT_IDLE;
									// Go to idle
									FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
									FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
								}
							}
						}
					}
					break;
				}

				default:
				{
					FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
					break;
				}
			}
			break;
		}
		default:
		{
			FCFlightData_local->ui32FlightStateMachine = FLIGHT_IDLE;
			FCFlightData_local->ui32FlightInitState = FINIT_IDLE;
			FCFlightData_local->ui32FlightDeInitStates = FDEINIT_IDLE;
			break;
		}
	}
}

// Stabilize according to data
void flight_stabilize(FLIGHT_CORE * FCFlightData_local)
{
	float32_t f32Error = 0.0f;
	float32_t f32IntegrationInterval = 0.01f;
	// Use PID regulator to stabilize vehicle
	// Stabilize roll - calculate error
	f32Error = FCFlightData_local->ORIENTATION_REQUIRED.f32Roll - FCFlightData_local->ORIENTATION.f32Roll;
	// Run PID
	math_PID(f32Error, f32IntegrationInterval, &FCFlightData_local->PIDRoll);

	// Stabilize pitch - calculate error
	f32Error = FCFlightData_local->ORIENTATION_REQUIRED.f32Pitch - FCFlightData_local->ORIENTATION.f32Pitch;
	// Run PID
	math_PID(f32Error, f32IntegrationInterval, &FCFlightData_local->PIDPitch);


	if(1 == FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID)
	{
		// Stabilize yaw - calculate error
		f32Error = FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw - FCFlightData_local->ORIENTATION.f32Yaw;
		// Run PID
		math_PID(f32Error, f32IntegrationInterval, &FCFlightData_local->PIDYaw);
	}

	// Run altitude PID
	// Calculate altitude diff from start
	FCFlightData_local->ORIENTATION.f32AltitudeAboveStart = FCFlightData_local->ORIENTATION.f32Altitude - FCFlightData_local->ORIENTATION.f32ZeroAltitude;

	// Filter altitude
	FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFilteredAcc += FCFlightData_local->ORIENTATION.f32AltitudeAboveStart;
	FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFiltered = FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFilteredAcc / FCFlightData_local->ORIENTATION.ui32AltitudeFilterWindow;
	FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFilteredAcc -= FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFiltered;
	// Error is required altitude - alt difference from start point
	f32Error = FCFlightData_local->ORIENTATION_REQUIRED.f32AltitudeAboveStart - FCFlightData_local->ORIENTATION.f32AltitudeAboveStartFiltered;
	math_PID(f32Error, f32IntegrationInterval, &FCFlightData_local->PIDAltitude);

	// Run speed PID

}

// Hardware specific function - decode flight commands to servo/motor commands
void flight_decodeServosY(FLIGHT_CORE * FCFlightData_local, RCDATA * RCValues)
{
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
		RCValues->ch[RC_AILERON_L].PWMOUT_Val = RCValues->ch[RC_AILERON_L].PWMOUT_Offset;
		RCValues->ch[RC_AILERON_R].PWMOUT_Val = RCValues->ch[RC_AILERON_R].PWMOUT_Offset;

		// Power is altitude output
		//f32Power = FCFlightData_local->PIDAltitude.outMax;
		// Power is throttle value + altitude PID output
		if(-50.0f > RCValues->ch[RC_THROTTLE].PWMIN_Zero)
		{
			FCFlightData_local->MOTORS.f32TotalPower = RCValues->f32ThrottleValue + FCFlightData_local->PIDAltitude.s;
		}
		else
		{
			FCFlightData_local->MOTORS.f32TotalPower = RCValues->f32ThrottleValue;
		}

		// Calculate motor power ratios
		// Motor FR
		f32Temp = FCFlightData_local->f32COGFrontFactor + FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f - FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Check min PWM
		if(MOTOR_MIN_PWMOUT > f32Temp)f32Temp = MOTOR_MIN_PWMOUT;
		// Store
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = f32Temp;

		// Motor FL
		f32Temp = FCFlightData_local->f32COGFrontFactor + FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f + FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Check min PWM
		if(MOTOR_MIN_PWMOUT > f32Temp)f32Temp = MOTOR_MIN_PWMOUT;
		// Store
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = f32Temp;

		// Motor RR
		f32Temp = FCFlightData_local->f32COGRearFactor - FCFlightData_local->PIDPitch.s;
		f32Temp1 = cosf(FCFlightData_local->PIDYaw.s);
		f32Temp /= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// And out PWM value
		f32Temp *= 1000;
		// Add zero PWM
		f32Temp += 1000;
		// Check min PWM
		if(MOTOR_MIN_PWMOUT > f32Temp)f32Temp = MOTOR_MIN_PWMOUT;
		// Store
		RCValues->ch[RC_MOTOR_RR].PWMOUT_Val = f32Temp;

		// Send values
		//CAN_SendRPM(2000, 2000, 0);

/*
		// Distribute power from RC command
		f32Temp = RCValues->ch[RC_ELEVATOR].PWMIN_Zero / 4000;
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 0.55f + f32Temp;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 0.55f + f32Temp;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = 0.45f - f32Temp;

		f32Temp = RCValues->ch[RC_AILERON].PWMIN_Zero / 4000;
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val -= f32Temp;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val += f32Temp;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val *= f32Power;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val *= f32Power;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val *= f32Power;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_FR].PWMOUT_Val);
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_FL].PWMOUT_Val);
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_R].PWMOUT_Val);

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val *= 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val *= 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val *= 1000;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val += 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val += 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val += 1000;

		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_FR].PWMOUT_Val)RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = MOTOR_MIN_PWMOUT;
		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_FL].PWMOUT_Val)RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = MOTOR_MIN_PWMOUT;
		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_R].PWMOUT_Val)RCValues->ch[RC_MOTOR_R].PWMOUT_Val = MOTOR_MIN_PWMOUT;
*/

		// Control yaw with nacelle roll
		// Let's assume that 1000 is 90 deg
		// So 1 deg is 11.11 q
		// Mult by 57,324840764331210191082802547771 to get from rad to deg
		// Together 636,942...
		f32Temp = FCFlightData_local->PIDYaw.s * 636.94267516f;

		// Check command
		if(0 != FCFlightData_local->f32YawCommand)
		{
			// Command input, set required yaw to current yaw and add command
			FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = FCFlightData_local->ORIENTATION.f32Yaw;
			f32Temp += FCFlightData_local->f32YawCommand;
			FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID = 0;
		}
		else
		{
			if(0 == FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID)
			{
				FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw += FCFlightData_local->f32YawCommandStartError;
			}
			FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID = 1;
			FCFlightData_local->f32YawCommandStartError = FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw - FCFlightData_local->ORIENTATION.f32Yaw;
		}

		// Limit nacelle roll
		if(MAX_NACELLE_ROLL < f32Temp) f32Temp = MAX_NACELLE_ROLL;
		else if(-MAX_NACELLE_ROLL > f32Temp) f32Temp = -MAX_NACELLE_ROLL;
		// Add zero PWM
		f32Temp += RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Offset;
		RCValues->i16YawValue = f32Temp;

		RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;

		// Disable motor PWM
/*
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = 1000;
*/
		//RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = 1500.0f + RCValues->ch[RC_RUDDER].PWMIN_Zero;

		/*

		if(RCValues->i16YawValue > RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
		{
			RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val += 5.0f;
			if(RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val > RCValues->i16YawValue) RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;
		}
		else if(RCValues->i16YawValue < RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
		{
			RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val -= 5.0f;
			if(RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val < RCValues->i16YawValue) RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;
		}

		RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = 1500.0f;
*/
	}
	else
	{

	}

	//***********************************
	// Set tilt values
	// Set new position
	// Servo FR position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.FR.f32ServoZero + FCFlightData_local->f32NacelleTilt_FR;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.FR.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.FR.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.FR.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
	}
	// Servo FL position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.FL.f32ServoZero + FCFlightData_local->f32NacelleTilt_FL;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.FL.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.FL.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.FL.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
	}
	// Servo R position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.RR.f32ServoZero + FCFlightData_local->f32NacelleTilt_RR;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.RR.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.RR.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.RR.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
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

// Hardware specific function - decode flight commands to servo/motor commands
void flight_decodeServosQ(FLIGHT_CORE * FCFlightData_local, RCDATA * RCValues)
{
	float32_t f32Temp;
	float32_t f32Temp1;
	int i = 0;
	// Next depends on current mode
	if(RC_Flags.bits.HOVER)
	{
		// We are in hover mode, all maneuvering is done with motors, flaps/ailerons to middle
		RCValues->ch[RC_AILERON_L].PWMOUT_Val = RCValues->ch[RC_AILERON_L].PWMOUT_Offset;
		RCValues->ch[RC_AILERON_R].PWMOUT_Val = RCValues->ch[RC_AILERON_R].PWMOUT_Offset;

		// Power is altitude output
		//f32Power = FCFlightData_local->PIDAltitude.outMax;
		// Power is throttle value + altitude PID output
		if(-50.0f > RCValues->ch[RC_THROTTLE].PWMIN_Zero)
		{
			FCFlightData_local->MOTORS.f32TotalPower = RCValues->f32ThrottleValue + FCFlightData_local->PIDAltitude.s;
		}
		else
		{
			FCFlightData_local->MOTORS.f32TotalPower = RCValues->f32ThrottleValue;
		}

		//enableMotors(1);



		// Calculate motor power ratios
		// Motor FR
		f32Temp = FCFlightData_local->f32COGFrontFactor + FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f - FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// Get correct RPM value
		f32Temp1 = (float)(FCFlightData_local->MOTORS.FR.i16MaxRPM - FCFlightData_local->MOTORS.FR.i16MinRPM);
		f32Temp1 *= f32Temp;
		FCFlightData_local->MOTORS.FR.i16ReqRPM = (int16_t)f32Temp1;
		FCFlightData_local->MOTORS.FR.fReqRPM = f32Temp1;

		// Motor FL
		f32Temp = FCFlightData_local->f32COGFrontFactor + FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f + FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// Get correct RPM value
		f32Temp1 = (float)(FCFlightData_local->MOTORS.FL.i16MaxRPM - FCFlightData_local->MOTORS.FL.i16MinRPM);
		f32Temp1 *= f32Temp;
		FCFlightData_local->MOTORS.FL.i16ReqRPM = (int16_t)f32Temp1;
		FCFlightData_local->MOTORS.FL.fReqRPM = f32Temp1;

		// Motor RR
		f32Temp = FCFlightData_local->f32COGRearFactor - FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f - FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// Get correct RPM value
		f32Temp1 = (float)(FCFlightData_local->MOTORS.RR.i16MaxRPM - FCFlightData_local->MOTORS.RR.i16MinRPM);
		f32Temp1 *= f32Temp;
		FCFlightData_local->MOTORS.RR.i16ReqRPM = (int16_t)f32Temp1;
		FCFlightData_local->MOTORS.RR.fReqRPM = f32Temp1;

		// Motor RL
		f32Temp = FCFlightData_local->f32COGRearFactor - FCFlightData_local->PIDPitch.s;
		f32Temp1 = 0.5f + FCFlightData_local->PIDRoll.s;
		f32Temp *= f32Temp1;
		// Get power
		f32Temp *= FCFlightData_local->MOTORS.f32TotalPower;
		// And sqrtf to get RPM
		f32Temp = sqrtf(f32Temp);
		// Get correct RPM value
		f32Temp1 = (float)(FCFlightData_local->MOTORS.RL.i16MaxRPM - FCFlightData_local->MOTORS.RL.i16MinRPM);
		f32Temp1 *= f32Temp;


		FCFlightData_local->MOTORS.RL.i16ReqRPM = (int16_t)f32Temp1;
		FCFlightData_local->MOTORS.RL.fReqRPM = f32Temp1;



/*
		// Distribute power from RC command
		f32Temp = RCValues->ch[RC_ELEVATOR].PWMIN_Zero / 4000;
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 0.55f + f32Temp;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 0.55f + f32Temp;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = 0.45f - f32Temp;

		f32Temp = RCValues->ch[RC_AILERON].PWMIN_Zero / 4000;
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val -= f32Temp;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val += f32Temp;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val *= f32Power;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val *= f32Power;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val *= f32Power;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_FR].PWMOUT_Val);
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_FL].PWMOUT_Val);
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = sqrtf(RCValues->ch[RC_MOTOR_R].PWMOUT_Val);

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val *= 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val *= 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val *= 1000;

		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val += 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val += 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val += 1000;

		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_FR].PWMOUT_Val)RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = MOTOR_MIN_PWMOUT;
		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_FL].PWMOUT_Val)RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = MOTOR_MIN_PWMOUT;
		if(MOTOR_MIN_PWMOUT > RCValues->ch[RC_MOTOR_R].PWMOUT_Val)RCValues->ch[RC_MOTOR_R].PWMOUT_Val = MOTOR_MIN_PWMOUT;
*/

		// Control yaw with nacelle roll
		// Let's assume that 1000 is 90 deg
		// So 1 deg is 11.11 q
		// Mult by 57,324840764331210191082802547771 to get from rad to deg
		// Together 636,942...
		f32Temp = FCFlightData_local->PIDYaw.s * 636.94267516f;

		// Check command
		if(0 != FCFlightData_local->f32YawCommand)
		{
			// Command input, set required yaw to current yaw and add command
			FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw = FCFlightData_local->ORIENTATION.f32Yaw;
			f32Temp += FCFlightData_local->f32YawCommand;
			FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID = 0;
		}
		else
		{
			if(0 == FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID)
			{
				FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw += FCFlightData_local->f32YawCommandStartError;
			}
			FCFlightData_local->ORIENTATION_REQUIRED.ui8CalculatePID = 1;
			FCFlightData_local->f32YawCommandStartError = FCFlightData_local->ORIENTATION_REQUIRED.f32Yaw - FCFlightData_local->ORIENTATION.f32Yaw;
		}

		// Limit nacelle roll
		if(MAX_NACELLE_ROLL < f32Temp) f32Temp = MAX_NACELLE_ROLL;
		else if(-MAX_NACELLE_ROLL > f32Temp) f32Temp = -MAX_NACELLE_ROLL;
		// Add zero PWM
		f32Temp += RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Offset;
		RCValues->i16YawValue = f32Temp;

		RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;

		// Disable motor PWM
/*
		RCValues->ch[RC_MOTOR_FR].PWMOUT_Val = 1000;
		RCValues->ch[RC_MOTOR_FL].PWMOUT_Val = 1000;
		RCValues->ch[RC_MOTOR_R].PWMOUT_Val = 1000;
*/
		//RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = 1500.0f + RCValues->ch[RC_RUDDER].PWMIN_Zero;

		/*

		if(RCValues->i16YawValue > RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
		{
			RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val += 5.0f;
			if(RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val > RCValues->i16YawValue) RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;
		}
		else if(RCValues->i16YawValue < RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val)
		{
			RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val -= 5.0f;
			if(RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val < RCValues->i16YawValue) RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = RCValues->i16YawValue;
		}

		RCValues->ch[RC_MOTOR_R_TILT].PWMOUT_Val = 1500.0f;
*/
	}
	else
	{

	}

	//***********************************
	// Set tilt values
	// Set new position
	// Servo FR position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.FR.f32ServoZero + FCFlightData_local->f32NacelleTilt_FR;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.FR.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.FR.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.FR.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
	}
	// Servo FL position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.FL.f32ServoZero + FCFlightData_local->f32NacelleTilt_FL;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.FL.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.FL.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.FL.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
	}
	// Servo R position*****************
	f32Temp = FCFlightData_local->TILT_SERVOS.RR.f32ServoZero + FCFlightData_local->f32NacelleTilt_RR;
	f32Temp *= 11.37777777777777;
	if(0 == FCFlightData_local->TILT_SERVOS.RR.ui8Reverse)
	{
		FCFlightData_local->TILT_SERVOS.RR.ui16RequestedPosition = (uint16_t)f32Temp;
	}
	else
	{
		FCFlightData_local->TILT_SERVOS.RR.ui16RequestedPosition = 4096 - (uint16_t)f32Temp;
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


