/*
 * flight.h
 *
 *  Created on: 8. feb. 2015
 *      Author: Jure
 */

#ifndef FLIGHT_H_
#define FLIGHT_H_

#include "flight_typedefs.h"
#include "math/myMath_typedefs.h"

// Functions
void flight_init(FLIGHT_CORE *data, RCDATA * RCInputs);
void flight_checkRCInputs(RCDATA * RCInputs, FLIGHT_CORE * FCFlightData);
void flight_checkStates(FLIGHT_CORE *data, RCDATA * RCValues);
void flight_stabilizeHover(FLIGHT_CORE * FCFlightData);
void flight_decodeServos(FLIGHT_CORE * FCFlightData, RCDATA * RCValues);

// Default values
// Min plane speed in m/sec
#define RC_DEFAULT_PLANE_MIN_SPEED		10.0f
#define RC_DEFAULT_HOVER_MAX_SPEED		5.0f
// Nacelle tilt speed in deg/iteration
#define RC_DEFAULT_TILT_SPEED			0.1f;
// Nacelle tilt in deg
#define RC_DEFAULT_NACELLE_TILT			90.0f;
// Default transition tilt in deg
#define RC_NACELLE_TRANSITION_TILT		45.0f;
#define RC_NACELLE_HOVERTRANSITION_TILT	75.0f;
// Define nacelle zeros
#define NACELLE_MAX_TILT_DEVIATION		2.0f
#define NACELLE_FR_ZERO					2048.0f
#define NACELLE_FL_ZERO					2048.0f
#define NACELLE_R_ZERO					2048.0f

// Macros that encode PWM inputs to specific channels
#define RC_AILERON		PWMIN_2_Zero
#define RC_ELEVATOR		PWMIN_3_Zero
#define RC_THROTTLE		PWMIN_4_Zero
#define RC_RUDDER		PWMIN_5_Zero
#define RC_GEAR_GYRO	PWMIN_6_Zero
#define RC_FLAPS_PITCH	PWMIN_7_Zero

// Macros that encode PWM outputs for different servos
#define RC_AILERON_L	PWMOUT_Val_1
#define RC_AILERON_R	PWMOUT_Val_2

#define RC_MOTOR_R_TILT	PWMOUT_Val_8
#define RC_MOTOR_FL		PWMOUT_Val_9
#define RC_MOTOR_FR		PWMOUT_Val_10
#define RC_MOTOR_R		PWMOUT_Val_11

#define RC_GEAR			PWMOUT_Val_12

/*
#define RC_NACELLE_FL	PWMOUT_Val_9
#define RC_NACELLE_FR	PWMOUT_Val_10
#define RC_NACELLE_BM	PWMOUT_Val_11
#define RC_NACELLE_BR	PWMOUT_Val_12
*/

#define RC_AILERON_L_MID	PWMOUT_Offset_1
#define RC_AILERON_R_MID	PWMOUT_Offset_2


#define RC_NACELLE_FL_MID	0
#define RC_NACELLE_FR_MID	0
#define RC_NACELLE_R_MID	0
#define RC_NACELLE_BR_MID	PWMOUT_Offset_12

// Input values macros
// Default midpoint
#define RC_IN_DEFAULT_MIDPOINT		1500.0f
// We are counting in microseconds
#define RC_IN_ZERO_VAL_OFFSET		10.0f
// Macros for switches
// Macro that defines midpoint for gear/gyro switch
#define RC_IN_GEAR_GYRO_MIDPOINT	1500.0f

// Default scales for RC inputs
#define RC_IN_DEFAULT_SCALE_AILERON		1.0f
#define RC_IN_DEFAULT_SCALE_ELEVATOR	1.0f
#define RC_IN_DEFAULT_SCALE_THROTTLE	1.0f
#define RC_IN_DEFAULT_SCALE_RUDDER		1.0f


// Macros
// States
#define FLIGHT_IDLE						0
#define FLIGHT_INIT						1
#define FLIGHT_STABILIZE_HOVER			2
#define FLIGHT_STABILIZE_PLANE			3
#define FLIGHT_STABILIZE_TRANSITION		4

// Flight init states
#define FINIT_IDLE						0
#define FINIT_MEAS_PWM_LOW				1
#define FINIT_MEAS_PWM_HIGH				2
#define FINIT_WAIT_STORQUE_ON			3
#define FINIT_WAIT_SPOS_DOWN			4
#define FINIT_WAIT_MOTOR_ON				5
#define FINIT_WAIT_MOTOR_PARK			6
#define FINIT_WAIT_SPOS_UP				7
#define FINIT_WAIT_SPOS_LEVEL			8
#define FINIT_WAIT_SPOS_VTOL			9

// Tilt transition states
#define FLIGHT_TILT_START				0
#define FLIGHT_TILT_P					1
#define FLIGHT_TILT_H					2
#define FLIGHT_TILT_TILTED_P			3
#define FLIGHT_TILT_TILTED_H			4
#define FLIGHT_TILT_SPEED_OK_P			5
#define FLIGHT_TILT_SPEED_OK_H			6
#define FLIGHT_TILT_END_P				7
#define FLIGHT_TILT_END_H				8
#define FLIGHT_TILT_ABORT_TILT_P		9
#define FLIGHT_TILT_ABORT_TILT_H		10
#define FLIGHT_TILT_ABORT_TILTED_P		11
#define FLIGHT_TILT_ABORT_TILTED_H		12
#define FLIGHT_TILT_ABORT_SPEED_OK_P	13
#define FLIGHT_TILT_ABORT_SPEED_OK_H	14
#define FLIGHT_TILT_ABORT_END_P			15
#define FLIGHT_TILT_ABORT_END_H			16


// Flight variables
extern RCFlag RC_Flags;


#endif /* FLIGHT_H_ */
