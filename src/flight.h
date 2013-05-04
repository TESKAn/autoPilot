/*
 * flight.h
 *
 *  Created on: May 4, 2013
 *      Author: Jure
 */

#ifndef FLIGHT_H_
#define FLIGHT_H_

// Definitions for structures
// Structure for floating point PI regulator
typedef struct
{
	uint32_t dataTime;
	// Proportional gain
	float32_t Kp;
	// Proportional variables
	float32_t P;
	// Integral gain
	float32_t Ki;
	// Integration variables
	float32_t I;
	// Regulator out values
	float32_t R;
	// Integral max value
	float32_t maxI;
	// Integral min value
	float32_t minI;
	// Error min/max value
	float32_t eMin;
	float32_t eMax;
	// Result min/max value
	float32_t rMin;
	float32_t rMax;
}__attribute__((aligned(4),packed)) PIData;

// Variable extern
extern PIData RollPID;
extern PIData PitchPID;
extern PIData YawPID;

// Variable default values
#define FLIGHT_DEFAULT_KP		1
#define FLIGHT_DEFAULT_KI		1
#define FLIGHT_DEFAULT_MAXI		5
#define FLIGHT_DEFAULT_MINI		-5
#define FLIGHT_DEFAULT_EMAX		5
#define FLIGHT_DEFAULT_EMIN		-5
#define FLIGHT_DEFAULT_RMAX		5
#define FLIGHT_DEFAULT_RMIN		-5


// Function declarations
#ifdef __cplusplus
 extern "C" {
#endif

 ErrorStatus FlightPID(PIData* PID, float32_t error, uint32_t deltaT);
 void initFlight(void);
 void initPID(PIData* PID);

#ifdef __cplusplus
}
#endif


#endif /* FLIGHT_H_ */
