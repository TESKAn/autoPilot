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

// Parameter variables
float32_t param_min_airspeed = DEF_MIN_AIRSPEED;
float32_t param_min_rotation = DEF_MIN_ROTATION;
float32_t param_min_rot_error = DEF_MIN_ROT_ERROR;
