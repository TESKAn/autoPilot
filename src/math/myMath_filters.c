/*
 * myMath_filters.c
 *
 *  Created on: 9. apr. 2016
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "allinclude.h"
#include "myMath_typedefs.h"
#include "myMath.h"

ErrorStatus math_filterInit(myMath_filter * filter, float32_t initialValue, float32_t window)
{
	filter->window = window;
	filter->filter_acc = filter->window * initialValue;
	filter->filter_acc-= initialValue;

	return SUCCESS;
}

float32_t math_filterUpdate(myMath_filter * filter, float32_t rawValue)
{
	// Add value
	filter->filter_acc += rawValue;
	// Calculate result
	filter->filter_result = filter->filter_acc / filter->window;
	// Remove result
	filter->filter_acc -= filter->filter_result;

	return filter->filter_result;
}

ErrorStatus math_filter3Update(myMath_filter3 *filter, Vectorf *rawValue)
{
	filter->result.x = math_filterUpdate(&filter->X, rawValue->x);
	filter->result.y = math_filterUpdate(&filter->Y, rawValue->y);
	filter->result.z = math_filterUpdate(&filter->Z, rawValue->z);

	return SUCCESS;
}
