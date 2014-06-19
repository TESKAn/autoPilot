/*
 * kalman.c
 *
 *  Created on: 16. jun. 2014
 *      Author: Jure
 */
#include "stm32f4xx.h"
#include "arm_math.h"
#include "kalman.h"

void Kalman_Init(KALMAN* data, float32_t Q, float32_t R)
{
	data->Q = Q;
	data->R = R;
	data->P_last = 0;
	data->x_est_last = 0;
}

float32_t Kalman_Update(KALMAN* data, float32_t newMeasurement)
{
	// Do prediction
	//do a prediction
	data->x_temp_est = data->x_est_last;
	data->P_temp = data->P_last + data->Q;
	//calculate the Kalman gain
	data->K = data->P_temp * (1.0f/(data->P_temp + data->R));

	// Correct
	data->x_est = data->x_temp_est + data->K * (newMeasurement - data->x_temp_est);
	data->P = (1- data->K) * data->P_temp;

	// Update
	//update our last's
	data->P_last = data->P;
	data->x_est_last = data->x_est;
	return data->x_est;
}
