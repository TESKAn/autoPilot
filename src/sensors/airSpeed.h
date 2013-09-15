/*
 * airSpeed.h
 *
 *  Created on: Aug 18, 2013
 *      Author: Jure
 */

#ifndef AIRSPEED_H_
#define AIRSPEED_H_

typedef struct
{
	uint8_t valid;
	float32_t airSpeed;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
}__attribute__((aligned(4),packed)) airSpeedData;

extern airSpeedData _AirSpeed;

// Function declarations
ErrorStatus AirSpeed_initDataStructure(void);
ErrorStatus AirSpeed_CalculateAirSpeed(uint16_t Pp, float32_t Pb, uint16_t T);

#endif /* AIRSPEED_H_ */