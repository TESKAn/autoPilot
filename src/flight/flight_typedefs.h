/*
 * flight_typedefs.h
 *
 *  Created on: 8. feb. 2015
 *      Author: Jure
 */

#ifndef FLIGHT_TYPEDEFS_H_
#define FLIGHT_TYPEDEFS_H_

#include "math/myMath_typedefs.h"
//#include "myMath_typedefs.h"



// Flag register typedef
typedef union
{
	struct
	{
		volatile uint32_t flag;
	}flag;

	 struct
	 {
		volatile uint8_t HOVER:1;
		volatile uint8_t PLANE:1;
		volatile uint8_t TRANSITION:1;
		volatile uint8_t ARMED:1;
		volatile uint8_t BIT4:1;
		volatile uint8_t BIT5:1;
		volatile uint8_t BIT6:1;
		volatile uint8_t BIT7:1;
		volatile uint8_t BIT8:1;
		volatile uint8_t BIT9:1;
		volatile uint8_t BIT10:1;
		volatile uint8_t BIT11:1;
		volatile uint8_t BIT12:1;
		volatile uint8_t BIT13:1;
		volatile uint8_t BIT14:1;
		volatile uint8_t BIT15:1;
		volatile uint8_t BIT16:1;
		volatile uint8_t BIT17:1;
		volatile uint8_t BIT18:1;
		volatile uint8_t BIT19:1;
		volatile uint8_t BIT20:1;
		volatile uint8_t BIT21:1;
		volatile uint8_t BIT22:1;
		volatile uint8_t BIT23:1;
		volatile uint8_t BIT24:1;
		volatile uint8_t BIT25:1;
		volatile uint8_t BIT26:1;
		volatile uint8_t BIT27:1;
		volatile uint8_t BIT28:1;
		volatile uint8_t BIT29:1;
		volatile uint8_t BIT30:1;
		volatile uint8_t BIT31:1;
	 }bits;
}__attribute__((aligned(4),packed)) RCFlag;

typedef struct
{
	uint8_t ui8MotorID;
	uint8_t ui8Enable;
	uint8_t ui8Enabled;
	uint8_t ui8Park;
	uint8_t ui8Parked;
	uint8_t ui8ParkPosition;
	uint8_t ui8MeasPWMMin;
	uint8_t ui8MeasPWMMax;
	uint8_t ui8MeasuringPWMMin;
	uint8_t ui8MeasuringPWMMax;
	uint8_t ui8UsePWM;
	uint8_t ui8UsingPWM;
	uint8_t ui8ReverseRotation;
	uint8_t ui8Reversed;
	uint8_t ui8Empty[2];
	int16_t i16PWMMin;
	int16_t i16PWMMax;
	int16_t i16SetRPM;
	int16_t i16CurrentRPM;
}__attribute__((aligned(4),packed)) FLIGHT_MOTOR;

typedef struct
{
	float32_t f32ServoZero;
	float32_t f32ServoAngle;
	uint16_t ui16RequestedPosition;
	uint16_t ui16Empty;
	uint8_t ui8ServoID;
	uint8_t ui8Enable;
	uint8_t ui8Enabled;
	uint8_t ui8Reverse;
}__attribute__((aligned(4),packed)) FLIGHT_SERVO;

typedef struct
{
	uint32_t ui32FlightStateMachine;
	uint32_t ui32FlightTransitionState;

	uint32_t ui32FlightInitState;
	uint32_t ui32FlightDeInitStates;

	// PIDs
	myMath_PID PIDPitch;
	myMath_PID PIDRoll;
	myMath_PID PIDYaw;
	myMath_PID PIDAltitude;
	myMath_PID PIDSpeed;

	// Input data
	// Current vehicle orientation
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Altitude;
		float32_t f32Speed;
		float32_t f32ZeroAltitude;
	}ORIENTATION;
	// Required orientation
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Power;
		float32_t f32Altitude;
		float32_t f32Speed;
	}ORIENTATION_REQUIRED;

	struct
	{
		float32_t f32RollLimit;
		float32_t f32PitchLimit;
		float32_t f32YawLimit;
	}ORIENTATION_LIMITS;

	// Output data
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
	}ORIENTATION_ADJUST;

	struct
	{
		float32_t f32AllowedPositionDeviation;
		FLIGHT_SERVO FR;
		FLIGHT_SERVO FL;
		FLIGHT_SERVO R;
	}TILT_SERVOS;

	struct
	{
		FLIGHT_MOTOR FR;
		FLIGHT_MOTOR FL;
		FLIGHT_MOTOR R;
	}MOTORS;

	struct
	{
		uint16_t ui16GearUp;
		uint16_t ui16Empty;
	}GEAR;

	// Variables
	// Minimal plane speed to be considered as flying
	float32_t f32MinPlaneSpeed;
	// Maximum speed for hover transition
	float32_t f32MaxHoverSpeed;
	// Nacelle transition angle, keep them at this until we reach transition speed
	float32_t f32NacellePlaneTransitionTilt;
	// Transition angle for plane -> hover
	float32_t f32NacelleHoverTransitionTilt;
	// Engine nacelles tilt angle
	float32_t f32NacelleCommonTilt;
	float32_t f32NacelleTilt_FL;
	float32_t f32NacelleTilt_FR;
	float32_t f32NacelleTilt_R;
	// Engine nacelles tilt angle change for each iteration
	float32_t f32NacelleTiltSpeed;
}__attribute__((aligned(4),packed)) FLIGHT_CORE;


typedef struct
{
	uint16_t PWMIN;					// PWM value as read from timer
	uint16_t PWMOUT;				// PWM out value as set to timer
	float32_t PWMMax;				// Maximum measured PWM input value
	float32_t PWMMin;				// Minimum measured PWM input value
	float32_t PWMDiff;				// Difference between max and min value
	float32_t PWMIN_Zero;			// Input PWM value after offset removal
	float32_t PWMIN_MID;			// PWM input midpoint
	float32_t PWMOUT_Val;			// Calculated PWM output value
	float32_t PWMOUT_Offset;		// Output offset
	uint8_t PWM_Good;				// PWM input is active (receiving signal)
	uint8_t PWM_Timeout;			// PWM input timeout counter
	uint8_t empty[2];
}__attribute__((aligned(4),packed)) RC_CHANNEL;


// Structure that holds R/C signals
// RC PWM in typedef
typedef struct
{
	// Scales from R/C input to whatever we need
	struct
	{
		float32_t f32AileronScale;
		float32_t f32ElevatorScale;
		float32_t f32ThrottleScale;
		float32_t f32RudderScale;
	}SCALES;

	int16_t i16YawValue;
	int16_t i16ostanek;

	RC_CHANNEL ch[12];

	uint16_t RSSI;
	uint16_t ostanek;
	uint8_t inputs_ok;
	uint8_t ostanek1[3];

}__attribute__((aligned(4),packed)) RCDATA, *PRCDATA;

#endif /* FLIGHT_TYPEDEFS_H_ */
