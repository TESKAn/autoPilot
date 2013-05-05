/*
 * flight.c
 *
 *  Created on: May 4, 2013
 *      Author: Jure
 */

#include "allinclude.h"

// Variables
// PID regulators
PIData RollPID;
PIData PitchPID;
PIData YawPID;

// PWM outputs
uint16_t PWMValues[12];

// Stabilize flight function
// When remote control is idle, control roll, pitch, yaw to stabilize plane in level flight
ErrorStatus FlightStabilize(void)
{
	return SUCCESS;
}

// Return to launch function
// When remote signal is lost, return plane to launch
ErrorStatus FlightRTL(void)
{
	return SUCCESS;
}

// Circle function
// Circle plane

// Flight PID function
ErrorStatus FlightPID(PIData* PID, float32_t error, uint32_t deltaT)
{
	float32_t temp = 0;
	float32_t time = (float32_t)deltaT / 100;
	// Limit error
	error = ahrs_limitFloat(error, PID->eMax, PID->eMin);

	// Update PID
	// Calculate error I add
	temp = error * PID->Ki * time * SYSTIME_TOSECONDS;
	// Add
	PID->I = PID->I + temp;
	// Limit I part
	PID->I = ahrs_limitFloat(PID->I, PID->maxI, PID->minI);
	// Calculate error P add
	temp = error * PID->Kp;
	PID->P = temp;
	// Add to PI result
	PID->R = PID->P + PID->I;
	// Limit
	ahrs_limitFloat(PID->R, PID->rMax, PID->rMin);

	return SUCCESS;
}

// Function to initialize flight data
void initFlight(void)
{
	// Init variables to default values
	initPID(&RollPID);
	initPID(&PitchPID);
	initPID(&YawPID);
}

// Function to initialize PID variable
void initPID(PIData* PID)
{
	PID->dataTime = 0;
	// Proportional gain
	PID->Kp = FLIGHT_DEFAULT_KP;
	// Proportional variables
	PID->P = 0;
	// Integral gain
	PID->Ki = FLIGHT_DEFAULT_KI;
	// Integration variables
	PID->I = 0;
	// Regulator out values
	PID->R = 0;
	// Integral max value
	PID->maxI = FLIGHT_DEFAULT_MAXI;
	// Integral min value
	PID->minI = FLIGHT_DEFAULT_MINI;
	// Error min/max value
	PID->eMin = FLIGHT_DEFAULT_EMIN;
	PID->eMax = FLIGHT_DEFAULT_EMAX;
	// Result min/max value
	PID->rMin = FLIGHT_DEFAULT_RMIN;
	PID->rMax = FLIGHT_DEFAULT_RMAX;
}
