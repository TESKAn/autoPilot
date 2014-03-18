/*
 * sensor_typedefs.h
 *
 *  Created on: 14. mar. 2014
 *      Author: Jure
 */

#ifndef SENSOR_TYPEDEFS_H_
#define SENSOR_TYPEDEFS_H_

#include "math/myMath_typedefs.h"

typedef struct
{

	Vectorf vector;
	Vectorf vecorPrevious;	// Store previous result for use in offset removal
	Vectorf vectorRaw;
	Vectorf offset;			// Computed magnetometer offset
	Vectorf hardIron;
	Matrixf softIron;
	uint32_t dataTime;
	uint32_t deltaTime;		// Store time difference between current and previous sample
	float32_t magRate;
	float32_t sensorTemperature;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) MagData, *PMagData;

typedef struct
{
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf scale;
	Vectori16 offset;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t gyroRate;
	float32_t sensorTemperature;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) GyroData, *PGyroData;


typedef struct
{
	Vectorf speed3D;
	float32_t heading;
	// latitude
	float32_t latitude;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) GPSData, *PGPSData;

typedef struct
{
	float32_t pressure;
	float32_t altitude;
	float32_t temperature;
	uint32_t dataTime;
	uint32_t deltaTime;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) AltimeterData, *PAltimeterData;

typedef struct
{
	float32_t airSpeed;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) airSpeedData;

typedef struct
{
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf scale;
	Vectorf Speed_3D;
	Vectorf Speed_3D_Frac;
	float32_t speed_3D_dt;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t accRate;
	float32_t sensorTemperature;
	Vectori16 offset;
	uint8_t valid;
	uint8_t nerabim;
}__attribute__((aligned(4),packed)) AccelerometerData, *PAccelerometerData;

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

	// PIDs
	myMath_PID3 _gyroErrorPID;

	// DCM matrix
	Matrixf _fusion_DCM;

	// Fusion parameters
	struct
	{
		// Factor to recalculate systime in seconds
		float32_t systimeToSeconds;
		float32_t minRotation;
		float32_t minRotError;

	}PARAMETERS;

	// Time
	uint32_t dataTime;
	uint32_t deltaTime;
	// MPU6000 temperature
	float32_t MPUTemperature;


}__attribute__((aligned(4),packed)) FUSION_CORE, *PFUSION_CORE;

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
		int8_t baroTemperatureDegrees;
		uint8_t baroTemperatureFrac;
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
		int8_t baroTemperatureDegrees;
		uint8_t baroTemperatureFrac;
		uint32_t dataTakenTime;
		uint8_t padding[2];
	}arrays;
}__attribute__((aligned(4),packed)) FUSION_SENSORDATA, *PFUSION_SENSORDATA;

extern FUSION_CORE fusionData;

#endif /* SENSOR_TYPEDEFS_H_ */
