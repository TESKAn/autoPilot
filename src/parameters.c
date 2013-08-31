/*
 * parameters.c
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */

// Declarations of parameters used in program
// Also used to store parameters to SD card
#include "stm32f4xx.h"
#include "arm_math.h"
#include "parameters.h"

float32_t min_airspeed = DEF_MIN_AIRSPEED;
