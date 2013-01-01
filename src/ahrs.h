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

// Define vector components - columns
#define VECT_X		0
#define VECT_Y		1
#define VECT_Z		2

// Define vector components - rows
#define VECT_A		0
#define VECT_B		1
#define VECT_C		2

// Structure for vector with 3 elements
typedef struct
{
	uint32_t dataTime;
	VectType type;
	vector3f vector;
	float32_t vectorData[3];
}vectorData;


// Structure for 3x3 matrix
typedef struct
{
	uint32_t dataTime;
	vector3f vector;
	float32_t vectorData[9];
}matrix3by3;



// Data valid
typedef enum {INVALID = 0, VALID = !INVALID} GPSDataValid;
// Data structures
// GPS

typedef struct
{
	uint32_t dataTime;
	GPSDataValid dataValid;
	float latitude;
	float longitude;
	float speed;
	float altitude;
	float trackAngle;
}GPSTypeDef;



extern GPSTypeDef GPS_Data;
extern uint32_t GPS_DataReceivedTime;

// Gyro
extern vectorData GyroVector;
// Acceleration
extern vectorData AccelVector;
// Magnetometer
extern vectorData MagnetVector;
// Rotation matrix
extern matrix3by3 rotationMatrix;
// Temporary matrices and vectors
extern matrix3by3 tempMatrix;
extern matrix3by3 holdMatrix;
extern vectorData tempVector;

// Function exports - ahrs.h
arm_status updateRotationMatrix(matrix3by3 * rotMatrix, vectorData * rotVector);

// Function exports - ahrs_math.h
void updateScaledVector(vectorData * vector, uint16_t x, uint16_t y, uint16_t z, float rate);
void ahrs_vectorDataInit(vectorData * vector, VectType type);
void ahrs_vectorUpdate(vectorData * vector, float32_t i, float32_t j, float32_t k);
void ahrs_matrix3by3_init(matrix3by3 * matrix);
void ahrs_generate_rotationMatrix(matrix3by3 * matrix, float roll, float pitch, float yaw);
void ahrs_generate_rotationUpdateMatrix(vectorData * vectorA, matrix3by3 * matrix);
arm_status ahrs_mult_matrixes(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC);
arm_status ahrs_mult_vector_matrix(vectorData * vectorA, matrix3by3 * matrix, vectorData * vectorB);

#endif /* AHRS_H_ */
