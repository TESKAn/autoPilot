/*
 * airSpeed.h
 *
 *  Created on: Aug 18, 2013
 *      Author: Jure
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_




// Function declarations
ErrorStatus AirSpeed_initDataStructure(airSpeedData *data, uint32_t time);
ErrorStatus AirSpeed_CalculateAirSpeed(airSpeedData *data, uint16_t Pp, float32_t Pb, uint16_t T, uint32_t time);

#endif /* AIRSPEED_H_ */
