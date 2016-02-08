/*
 * gyro.h
 *
 *  Created on: Aug 19, 2013
 *      Author: Jure
 */

#ifndef GYRO_H_
#define GYRO_H_




ErrorStatus gyro_initDataStructure(GyroData *data, uint32_t time);
ErrorStatus gyro_update(FUSION_CORE *data, int16_t *rawData, uint32_t dataTime);

#endif /* GYRO_H_ */
