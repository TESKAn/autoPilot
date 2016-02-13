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
	uint32_t ui32FlightStateMachine;
	uint32_t ui32FlightTransitionState;

	uint32_t ui32FlightInitState;

	// PIDs
	myMath_PID PIDPitch;
	myMath_PID PIDRoll;
	myMath_PID PIDYaw;
	myMath_PID PIDAltitude;

	// Input data
	// Current vehicle orientation
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Altitude;
		float32_t f32Speed;
	}ORIENTATION;
	// Required orientation - plane
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Power;
	}ORIENTATION_REQUIRED_P;
	// Required orientation - hover
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Altitude;
		float32_t f32Speed;
	}ORIENTATION_REQUIRED_H;

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
		float32_t f32ServoFLZero;
		float32_t f32ServoFRZero;
		float32_t f32ServoRZero;
		float32_t f32ServoFRAngle;
		float32_t f32ServoFLAngle;
		float32_t f32ServoRAngle;
		uint16_t ui16FRRequestedPosition;
		uint16_t ui16FLRequestedPosition;
		uint16_t ui16RRequestedPosition;
		uint8_t ui8ServoFRID;
		uint8_t ui8ServoFLID;
		uint8_t ui8ServoRID;
		uint8_t ui8Empty;
		uint8_t ui8EnableFR;
		uint8_t ui8EnableFL;
		uint8_t ui8EnableR;
		uint8_t ui8Empty1;
		uint8_t ui8FREnabled;
		uint8_t ui8FLEnabled;
		uint8_t ui8REnabled;
		uint8_t ui8Empty2;
	}TILT_SERVOS;

	struct
	{
		uint8_t ui8MotorFRID;
		uint8_t ui8MotorFLID;
		uint8_t ui8MotorRID;
		uint8_t ui8Empty;
		uint8_t ui8EnableFR;
		uint8_t ui8EnableFL;
		uint8_t ui8EnableR;
		uint8_t ui8Empty1;
		uint8_t ui8FREnabled;
		uint8_t ui8FLEnabled;
		uint8_t ui8REnabled;
		uint8_t ui8Empty2;
		uint8_t ui8FRPark;
		uint8_t ui8FLPark;
		uint8_t ui8RPark;
		uint8_t ui8Empty3;
		uint8_t ui8FRParked;
		uint8_t ui8FLParked;
		uint8_t ui8RParked;
		uint8_t ui8Empty4;
		uint8_t ui8FRMeasPWMLow;
		uint8_t ui8FLMeasPWMLow;
		uint8_t ui8RMeasPWMLow;
		uint8_t ui8Empty5;
		uint8_t ui8FRMeasPWMHigh;
		uint8_t ui8FLMeasPWMHigh;
		uint8_t ui8RMeasPWMHigh;
		uint8_t ui8Empty6;
		uint8_t ui8FRUsePWM;
		uint8_t ui8FLUsePWM;
		uint8_t ui8RUsePWM;
		uint8_t ui8Empty7;
		uint8_t ui8FRReverseRotation;
		uint8_t ui8FLReverseRotation;
		uint8_t ui8RReverseRotation;
		uint8_t ui8Empty8;
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

	// Input raw values
	uint16_t PWMIN_1;
	uint16_t PWMIN_2;
	uint16_t PWMIN_3;
	uint16_t PWMIN_4;
	uint16_t PWMIN_5;
	uint16_t PWMIN_6;
	uint16_t PWMIN_7;
	uint16_t PWMIN_8;

	// Input value zeros
	float32_t PWMIN_1_Zero;
	float32_t PWMIN_2_Zero;
	float32_t PWMIN_3_Zero;
	float32_t PWMIN_4_Zero;
	float32_t PWMIN_5_Zero;
	float32_t PWMIN_6_Zero;
	float32_t PWMIN_7_Zero;
	float32_t PWMIN_8_Zero;

	// Input values after removing offset
	float32_t PWMIN_1_MID;
	float32_t PWMIN_2_MID;
	float32_t PWMIN_3_MID;
	float32_t PWMIN_4_MID;
	float32_t PWMIN_5_MID;
	float32_t PWMIN_6_MID;
	float32_t PWMIN_7_MID;
	float32_t PWMIN_8_MID;

	// Output values
	float32_t PWMOUT_Val_1;
	float32_t PWMOUT_Val_2;
	float32_t PWMOUT_Val_3;
	float32_t PWMOUT_Val_4;
	float32_t PWMOUT_Val_5;
	float32_t PWMOUT_Val_6;
	float32_t PWMOUT_Val_7;
	float32_t PWMOUT_Val_8;
	float32_t PWMOUT_Val_9;
	float32_t PWMOUT_Val_10;
	float32_t PWMOUT_Val_11;
	float32_t PWMOUT_Val_12;

	// Output offsets - add this to required value
	// to generate output centered around some zero value
	float32_t PWMOUT_Offset_1;
	float32_t PWMOUT_Offset_2;
	float32_t PWMOUT_Offset_3;
	float32_t PWMOUT_Offset_4;
	float32_t PWMOUT_Offset_5;
	float32_t PWMOUT_Offset_6;
	float32_t PWMOUT_Offset_7;
	float32_t PWMOUT_Offset_8;
	float32_t PWMOUT_Offset_9;
	float32_t PWMOUT_Offset_10;
	float32_t PWMOUT_Offset_11;
	float32_t PWMOUT_Offset_12;


	// Output values
	uint16_t PWMOUT_1;
	uint16_t PWMOUT_2;
	uint16_t PWMOUT_3;
	uint16_t PWMOUT_4;
	uint16_t PWMOUT_5;
	uint16_t PWMOUT_6;
	uint16_t PWMOUT_7;
	uint16_t PWMOUT_8;
	uint16_t PWMOUT_9;
	uint16_t PWMOUT_10;
	uint16_t PWMOUT_11;
	uint16_t PWMOUT_12;

	uint16_t RSSI;
	uint16_t ostanek;

}__attribute__((aligned(4),packed)) RCDATA, *PRCDATA;

#endif /* FLIGHT_TYPEDEFS_H_ */
