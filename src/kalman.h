/*
 * kalman.h
 *
 *  Created on: 16. jun. 2014
 *      Author: Jure
 */

#ifndef KALMAN_H_
#define KALMAN_H_

typedef struct
{
	float32_t x_est_last;
	float32_t P_last;
	float32_t Q;
	float32_t R;

	float32_t K;
	float32_t P;
	float32_t P_temp;
	float32_t x_temp_est;
	float32_t x_est;


}__attribute__((aligned(4),packed)) KALMAN, *PKALMAN;

void Kalman_Init(KALMAN* data, float32_t Q, float32_t R);
float32_t Kalman_Update(KALMAN* data, float32_t newMeasurement);

#endif /* KALMAN_H_ */
