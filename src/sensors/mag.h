/*
 * mag.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MAG_H_
#define MAG_H_





ErrorStatus mag_initDataStructure(MagData *data, uint32_t time);
ErrorStatus mag_update(FUSION_CORE *data, int16_t *rawData, uint32_t dataTime);

#endif /* MAG_H_ */
