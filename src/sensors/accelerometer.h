/*
 * accelerometer.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_




ErrorStatus acc_initDataStructure(AccelerometerData *data);
ErrorStatus acc_updateGains(AccelerometerData *data, int axis);
ErrorStatus acc_updateSpeedCalculation(FUSION_CORE *coreData, uint32_t dataTime);
ErrorStatus acc_update(FUSION_CORE *coreData, int16_t *rawData, uint32_t dataTime);

#endif /* ACCELEROMETER_H_ */
