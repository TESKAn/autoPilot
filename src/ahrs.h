/*
 * ahrs.h
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */

#ifndef AHRS_H_
#define AHRS_H_


// Vector definitions
typedef enum {ROW = 0, COLUMN = !ROW} VectType;
typedef arm_matrix_instance_f32 vector3f;
typedef arm_matrix_instance_q15 vector3q;

// Define vector components - columns
#define VECT_X		0
#define VECT_Y		1
#define VECT_Z		2

// Define vector components - rows
#define VECT_A		0
#define VECT_B		1
#define VECT_C		2

// Define R matrix elements
#define Rxx			0
#define Ryx			1
#define Rzx			2
#define Rxy			3
#define Ryy			4
#define Rzy			5
#define Rxz			6
#define Ryz			7
#define Rzz			8

// Structure for 3 floating point PI regulators
typedef struct
{
	uint32_t dataTime;
	// Proportional gain
	float32_t Kpx;
	float32_t Kpy;
	float32_t Kpz;
	// Integral gain
	float32_t Kix;
	float32_t Kiy;
	float32_t Kiz;
	// Integration variables
	float32_t Ix;
	float32_t Iy;
	float32_t Iz;
	// Regulator out values
	float32_t Rx;
	float32_t Ry;
	float32_t Rz;

}__attribute__((aligned(4),packed)) PI3Data;
// Structure for float vector with 3 elements
typedef struct
{
	float32_t vectorData[3];
	vector3f vector;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
	VectType type;
}__attribute__((aligned(4),packed)) vectorData;

// Structure for int16 vector with 3 elements
typedef struct
{
	int16_t vectorData[3];
	vector3q vector;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
	VectType type;
}__attribute__((aligned(4),packed)) vector3qData;

// Structure for 3x3 matrix
typedef struct
{
	float32_t vectorData[9];
	vector3f vector;
	uint32_t dataTime;
}__attribute__((aligned(4),packed)) matrix3by3;

// Data valid
typedef enum {INVALID = 0, VALID = !INVALID} GPSDataValid;
// Data structures
// GPS

typedef struct
{
	uint32_t dataTime;
	uint32_t dataStartTime;
	float latitude;
	float longitude;
	float speed;
	float altitude;
	float trackAngle;
	GPSDataValid dataValid;
}__attribute__((aligned(4),packed)) GPSTypeDef;

typedef struct
{
	// Rotation matrix
	matrix3by3 rotationMatrix;
	// Acceleration vector
	vectorData AccVector;
	// Acceleration offset vector
	vector3qData AccOffsetVector;
	// Acceleration scale vector
	vectorData AccScaleVector;
	// Gravity vector
	vectorData GravityVector;
	// Gyroscope vector
	vectorData GyroVector;
	// Gyroscope offsets
	vector3qData GyroOffsetVector;
	// Gyroscope scale vector
	vectorData GyroScaleVector;
	// Magnetometer vector
	vectorData MagVector;
	// Magnetometer offset vector
	vector3qData MagOffsetVector;
	// Magnetometer scale vector
	vectorData MagScaleVector;
	// Pitch, roll, yaw angles in rad
	vectorData RollPitchYaw;
	// GPS data
	GPSTypeDef GPSData;
	// GPS reference rotation matrix
	matrix3by3 GPSReference;
	// Speed data in m/sec
	float32_t PlaneSpeed;
	// PI data
	PI3Data PIData;
	// Scale values
	float32_t gyroRate;
	float32_t accRate;
	float32_t magRate;
}__attribute__((aligned(4),packed)) AHRSData;

extern AHRSData ahrs_data;


// Temporary matrices and vectors
extern matrix3by3 tempMatrix;
extern matrix3by3 holdMatrix;
extern vectorData tempVector;

// Function exports - ahrs.h
void initAHRSStructure(AHRSData * ahrsStructure);
arm_status ahrs_updateAccelerationToGyro(void);
arm_status ahrs_updateGPSToGyro(void);
arm_status ahrs_updateRotationMatrix(AHRSData * data);
void ahrs_resetRotationMatrix(void);
void ahrs_normalizeOrthogonalizeMatrix(matrix3by3 * rotMatrix);
void ahrs_getAngles(matrix3by3 * rotMatrix, vector3f *vector);

// Function exports - ahrs_math.h
void updateScaledVector(vectorData * vector, uint16_t x, uint16_t y, uint16_t z, float rate);
void ahrs_vectorDataInit(vectorData * vector, VectType type);
void ahrs_vector3qDataInit(vector3qData * vector, VectType type);
void ahrs_vectorUpdate(vectorData * vector, float32_t i, float32_t j, float32_t k);
void ahrs_matrix3by3_init(matrix3by3 * matrix);
void ahrs_generate_rotationMatrix(matrix3by3 * matrix, float roll, float pitch, float yaw);
void ahrs_generate_rotationUpdateMatrix(vectorData * vectorA, matrix3by3 * matrix);
arm_status ahrs_mult_matrixes(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC);
arm_status ahrs_mult_vector_matrix(vectorData * vectorA, matrix3by3 * matrix, vectorData * vectorB);

#endif /* AHRS_H_ */
