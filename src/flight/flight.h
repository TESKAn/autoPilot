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
void flight_init(FLIGHT_CORE *data);


// Macros
// States
#define FLIGHT_IDLE					0
#define FLIGHT_STABILIZE			1



#endif /* FLIGHT_H_ */
