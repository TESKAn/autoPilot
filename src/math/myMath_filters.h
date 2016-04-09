/*
 * myMath_filters.h
 *
 *  Created on: 9. apr. 2016
 *      Author: Jure
 */

#ifndef MYMATH_FILTERS_H_
#define MYMATH_FILTERS_H_

ErrorStatus math_filterInit(myMath_filter * filter, float32_t initialValue, float32_t window);
float32_t math_filterUpdate(myMath_filter * filter, float32_t rawValue);
ErrorStatus math_filter3Update(myMath_filter3 *filter, Vectorf *rawValue);

#endif /* MYMATH_FILTERS_H_ */
