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
void flight_checkStates(FLIGHT_CORE *data);


// Macros
// States
#define FLIGHT_IDLE						0
#define FLIGHT_STABILIZE_HOVER			1
#define FLIGHT_STABILIZE_PLANE			2
#define FLIGHT_STABILIZE_TRANSITION		3

// Flight variables
extern RCFlag RC_Flags;


#endif /* FLIGHT_H_ */
