/*
 * sensors_fusion.h
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#ifndef SENSORS_FUSION_H_
#define SENSORS_FUSION_H_

// Includes
#include "math/myMath_typedefs.h"

#include "gyro.h"
#include "accelerometer.h"
#include "mag.h"
#include "airSpeed.h"
#include "gps.h"
#include "altimeter.h"


// Core structure for sensor fusion data
typedef struct
{
	// Structs for individual sensors
	AccelerometerData _accelerometer;
	GyroData _gyro;
	MagData _mag;
	airSpeedData _airSpeed;
	AltimeterData _altimeter;
	GPSData _gps;

	// DCM matrix
	Matrixf _fusion_DCM;


}FUSION_CORE;

// Structure for sensor data read from sensors.
// 26 bytes used
// Modify to fit data from sensor
// MPU 6000/6050 with mag and pressure sensors
typedef union
{
	uint8_t buf[32];
	uint16_t buf16[16];
	struct
	{
		// Acceleration
		int16_t accX;	//0,1
		int16_t accY;	//2,3
		int16_t accZ;	//4,5
		// Temperature
		int16_t temperature;	//6,7
		// Gyro
		int16_t gyroX;	//8,9
		int16_t gyroY;	//10,11
		int16_t gyroZ;	//12,13
		int16_t magX;	//14,15
		int16_t magZ;	//16,17
		int16_t magY;	//18,19
		union
		{
			uint32_t statusPressure;
			struct
			{
				uint8_t status;
				uint8_t OUT_P_MSB;
				uint8_t OUT_P_CSB;
				uint8_t OUT_P_LSB;
			}parts;
		}pressure;
		int16_t baroTemperature;
		uint32_t dataTakenTime;
		uint8_t padding[2];
	}data;
	struct
	{
		int16_t acc[3];
		int16_t temperature;
		int16_t gyro[3];
		int16_t mag[3];
		union
		{
			uint32_t statusPressure;
			struct
			{
				uint8_t status;
				uint8_t OUT_P_MSB;
				uint8_t OUT_P_CSB;
				uint8_t OUT_P_LSB;
			}parts;
		}pressure;
		int16_t baroTemperature;
		uint32_t dataTakenTime;
		uint8_t padding[2];
	}arrays;
}FUSION_SENSORDATA;


ErrorStatus fusion_init(FUSION_CORE *coreData);
ErrorStatus fusion_dataUpdate(FUSION_CORE *coreData, FUSION_SENSORDATA *sensorData);
ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix);
ErrorStatus fusion_updateRotationMatrix(FUSION_CORE *data);

#endif /* SENSORS_FUSION_H_ */
