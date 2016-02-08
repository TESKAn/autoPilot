/*
 * altimeter.h
 *
 *  Created on: Sep 1, 2013
 *      Author: Jure
 */

#ifndef ALTIMETER_H_
#define ALTIMETER_H_




ErrorStatus altimeter_initDataStructure(AltimeterData *data, uint32_t time);
ErrorStatus altimeter_update(FUSION_CORE *data, uint32_t rawData_P, int8_t temp_deg, uint8_t temp_frac, uint32_t dataTime);

#endif /* ALTIMETER_H_ */
