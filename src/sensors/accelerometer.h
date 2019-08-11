/*
 * accelerometer.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

ErrorStatus acc_initDataStructure(AccelerometerData *data, uint32_t time);
ErrorStatus acc_updateOffsets(AccelerometerData *data);
ErrorStatus acc_updateGains(AccelerometerData *data, int axis);
ErrorStatus acc_updateSpeedCalculation(FUSION_CORE *coreData, uint32_t dataTime);

#endif /* ACCELEROMETER_H_ */
