/*
 * airSpeed.h
 *
 *  Created on: Aug 18, 2013
 *      Author: Jure
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_




// Function declarations
ErrorStatus AirSpeed_initDataStructure(airSpeedData *data);
ErrorStatus AirSpeed_CalculateAirSpeed(airSpeedData *data, uint16_t Pp, float32_t Pb, uint16_t T);

#endif /* AIRSPEED_H_ */
