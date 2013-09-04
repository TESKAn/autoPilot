/*
 * parameters.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// Definitions for parameters defaults
#define DEF_MIN_AIRSPEED		1.0f		// Minimum 1 m/s air speed to use GPS and accelerometer for drift error
#define DEF_MIN_ROTATION		0.0001f		// Minimal rotation to count as actual rotation
#define DEF_MIN_ROT_ERROR		0.0001f		// Min error to update rotation error PID

// Extern definitions
extern float32_t param_min_airspeed;
extern float32_t param_min_rotation;
extern float32_t param_min_rot_error;

#endif /* PARAMETERS_H_ */
