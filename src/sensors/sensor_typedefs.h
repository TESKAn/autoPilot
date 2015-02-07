/*
 * sensor_typedefs.h
 *
 *  Created on: 14. mar. 2014
 *      Author: Jure
 */

#ifndef SENSOR_TYPEDEFS_H_
#define SENSOR_TYPEDEFS_H_

#include "math/myMath_typedefs.h"
#include "kalman.h"

// Flag register typedef
typedef union
{
	struct
	{
		volatile uint32_t flag;
	}flag;

	 struct
	 {
		volatile uint8_t BIT0:1;
		volatile uint8_t BIT1:1;
		volatile uint8_t BIT2:1;
		volatile uint8_t BIT3:1;
		volatile uint8_t BIT4:1;
		volatile uint8_t BIT5:1;
		volatile uint8_t BIT6:1;
		volatile uint8_t BIT7:1;
		volatile uint8_t BIT8:1;
		volatile uint8_t BIT9:1;
		volatile uint8_t BIT10:1;
		volatile uint8_t BIT11:1;
		volatile uint8_t BIT12:1;
		volatile uint8_t BIT13:1;
		volatile uint8_t BIT14:1;
		volatile uint8_t BIT15:1;
		volatile uint8_t BIT16:1;
		volatile uint8_t BIT17:1;
		volatile uint8_t BIT18:1;
		volatile uint8_t BIT19:1;
		volatile uint8_t BIT20:1;
		volatile uint8_t BIT21:1;
		volatile uint8_t BIT22:1;
		volatile uint8_t BIT23:1;
		volatile uint8_t BIT24:1;
		volatile uint8_t BIT25:1;
		volatile uint8_t BIT26:1;
		volatile uint8_t BIT27:1;
		volatile uint8_t BIT28:1;
		volatile uint8_t BIT29:1;
		volatile uint8_t BIT30:1;
		volatile uint8_t BIT31:1;
	 }bits;
}__attribute__((aligned(4),packed)) SensorFlag, *PSensorFlag;

typedef struct
{
	Vectorf vector;					// Normalized vector in body frame
	Vectorf vectorRaw;
	Vectorf vectorKFiltered;
	Vectorf vectorEarthFrame;		// Normalized vector in earth frame
	Vectorf vecorPrevious;			// Store previous result for use in offset removal
	KALMAN3 kFilter;				// Kalman filter data
	float32_t heading;				// Heading calculated from mag in earth frame
	Vectorf earthYAxis;				// Earth's Y axis, as seen by magnetometer, in body frame
	Vectorf offset;					// Computed magnetometer offset
	Vectorf currentMagReading;		// Mag readings with only offset removed
	Vectorf previousMagReading;
	float32_t currentMagnitude;
	float32_t previousMagnitude;
	Vectorf hardIron;
	Vectorf calcVector;				// Used for calculations
	Matrixf softIron;
	uint32_t dataTime;
	uint32_t deltaTime;				// Store time difference between current and previous sample
	float32_t magRate;
	float32_t sensorTemperature;
	float32_t magOffsetNullGain;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) MagData, *PMagData;

typedef struct
{
	Vectorf vector;
	Vectorf vectorRaw;
	Vectorf vectorKFiltered;
	Vectorf scale;
	float32_t fDeltaTime;
	float32_t fReceiveTime;
	float32_t errorScale;
	// Previous rotation
	Vectorf vectorW_m;
	//
	Vectorf vectorW_p;
	// Kalman filter data
	KALMAN3 kFilter;
	// Error
	Vectorf gyroError;
	Vectorf gyroGainError;
	Vectori16 offset;
	Vectorf offsets;
	uint32_t dataTime;
	uint32_t deltaTime;
	float32_t gyroRate;
	float32_t gyroRateXP;
	float32_t gyroRateXN;
	float32_t gyroRateYP;
	float32_t gyroRateYN;
	float32_t gyroRateZP;
	float32_t gyroRateZN;
	float32_t sensorTemperature;
	uint8_t valid;
	uint8_t nerabim[3];
}__attribute__((aligned(4),packed)) GyroData, *PGyroData;


typedef struct
{
	// Use double for lat, lon
	float64_t latitude;
	float64_t longitude;

	float32_t speed;
	float32_t altitude;
	float32_t trackAngle;
	uint32_t dataTime;
	uint32_t deltaTime;
	Vectorf speed3D;
	float32_t hdop;
	float32_t gg;
	float32_t magvar;

	uint16_t satStatus;
	uint16_t miliseconds;
	uint16_t valid;
	uint16_t ns_ew;
	uint16_t magvar_ew;
	uint16_t buf;

	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t day;

	uint8_t month;
	uint8_t year;
	uint8_t dataOK;
	uint8_t nerabim;
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
	Vectorf vectorKFiltered;
	Vectorf vectorNormalized;
	KALMAN3 kFilter;				// Kalman filter data
	Vectorf gains;
	Vectorf offsets;
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

	// Maximum gyro amplitude when updating gyro error PID
	float32_t maxGyroErrorAmplitude;

	// Maximum allowed rotation when updating PID error
	float32_t maxGyroErrorUpdateRate;

	// When do we consider gyro to be rotating fast
	float32_t gyroFastRotation;

	// Update gain error when gyro rate above this value
	float32_t gyroGainUpdateRate;

	// PIDs
	myMath_PID3 _gyroErrorPID;

	myMath_PID3 _gyroGainPID;
	// Counter for error PID
	uint32_t _gyroErrorUpdateCount;
	uint32_t _gyroIErrorUpdateCount;

	// DCM matrix
	Matrixf _fusion_DCM;

	// Update rotation vector
	Vectorf updateRotation;

	// Integration time
	float32_t integrationTime;

	// GPS DCM matrix
	Matrixf _GPS_DCM;

	// Roll pitch yaw
	struct
	{
		float32_t roll;
		float32_t pitch;
		float32_t yaw;
	}ROLLPITCHYAW;

	// Fusion parameters
	struct
	{
		// Factor to recalculate systime in seconds
		float32_t systimeToSeconds;
		float32_t minRotation;
		float32_t minRotError;
		float32_t minGPSSpeed;
		uint32_t gyroErrorUpdateInterval;
		uint32_t gyroIErrorUpdateInterval;
	}PARAMETERS;

	// Stored data from previous rotation updates
	struct
	{
		Matrixf Phi[3];
		/*
		Matrixf Phi1;
		Matrixf Phi2;
		*/
		uint32_t Phi0Index;
	}ROTATIONS;

	// Time
	uint32_t dataTime;
	uint32_t deltaTime;
	uint32_t ui32SensorInitTime;
	// MPU6000 temperature
	float32_t MPUTemperature;

	uint8_t SelfTestValues[4];

	uint32_t sensorInterruptTime;
	uint32_t sensorInterruptDeltaTime;

	// Flag structure
	SensorFlag sFlag;

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
