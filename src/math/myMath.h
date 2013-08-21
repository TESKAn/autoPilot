/*
 * myMath.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Jure
 */

#ifndef MYMATH_H_
#define MYMATH_H_


float32_t math_fastInverseRoot(float32_t num);
myMath_PID math_PIDInit(float32_t kp, float32_t ki, float32_t kd);
myMath_PID3 math_PID3Init(float32_t kp, float32_t ki, float32_t kd);
ErrorStatus math_PIDReset(myMath_PID * PID);
ErrorStatus math_PID3Reset(myMath_PID3 * PID);
ErrorStatus math_PID(float32_t e, float32_t dt, myMath_PID * PID);
ErrorStatus math_PID3(Vectorf * error, float32_t dt, myMath_PID3 * PID);

#endif /* MYMATH_H_ */
