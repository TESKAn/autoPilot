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

// Macros that encode PWM inputs to specific channels
#define RC_AILERON		PWMIN_1_Zero
#define RC_ELEVATOR		PWMIN_2_Zero
#define RC_THROTTLE		PWMIN_3_Zero
#define RC_RUDDER		PWMIN_4_Zero
#define RC_GEAR_GYRO	PWMIN_5_Zero
#define RC_FLAPS_PITCH	PWMIN_6_Zero

// Input values macros
// We are counting in microseconds
#define RC_IN_ZERO_VAL_OFFSET	10.0f

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
		volatile uint8_t BIT1:1;
		volatile uint8_t BIT2:1;
		volatile uint8_t BIT3:1;
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

	// PIDs
	myMath_PID3 OrientationPIDs;

	// Input data
	// Plane orientation
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Altitude;
	}ORIENTATION;
	// Required orientation - plane
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32Power;
	}ORIENTATION_REQUIRED_P;
	// Required orientation - copter
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
		float32_t f32AltitudeChange;
	}ORIENTATION_REQUIRED_Q;

	// Output data
	struct
	{
		float32_t f32Roll;
		float32_t f32Pitch;
		float32_t f32Yaw;
	}ORIENTATION_ADJUST;


}__attribute__((aligned(4),packed)) FLIGHT_CORE;

// Structure that holds R/C signals
// RC PWM in typedef
typedef struct
{
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
