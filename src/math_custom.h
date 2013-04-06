/*
 * math_custom.h
 *
 *  Created on: Jan 13, 2013
 *      Author: Jure
 */

#ifndef MATH_CUSTOM_H_
#define MATH_CUSTOM_H_

typedef enum {MATH_NO = 0, MATH_YES = !MATH_NO} MathYesNo;

typedef arm_matrix_instance_f32 vector3f;
typedef arm_matrix_instance_q15 vector3q;

// Vector definitions
typedef enum {ROW = 0, COLUMN = !ROW} VectType;

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

// Structure for float quaternion
typedef struct
{
	float32_t w;
	float32_t x;
	float32_t y;
	float32_t z;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
}__attribute__((aligned(4),packed)) quaternion;

// Structure for float vector with 3 elements
typedef struct
{
	float32_t vector3fData[3];
	vector3f vector;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
	VectType type;
}__attribute__((aligned(4),packed)) vector3fData;

// Structure for int16 vector with 3 elements
typedef struct
{
	int16_t vector3fData[3];
	vector3q vector;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
	VectType type;
}__attribute__((aligned(4),packed)) vector3qData;

// Structure for 3x3 matrix
typedef struct
{
	float32_t vector3fData[9];
	vector3f vector;
	uint32_t dataTime;
	uint32_t deltaTime;		// Time that has passed between two samples
}__attribute__((aligned(4),packed)) matrix3by3;

float32_t math_float32Abs(float32_t x);
float32_t math_limitFloat(float32_t number, float32_t max, float32_t min);
void math_vector3fDataInit(vector3fData * vector, VectType type);
void math_vector3qDataInit(vector3qData * vector, VectType type);
void math_vectorUpdate(vector3fData * vector, float32_t i, float32_t j, float32_t k);
void math_matrix3by3Init(matrix3by3 * matrix, MathYesNo identity);
void math_generateRotationUpdateMatrix(float32_t x, float32_t y, float32_t z, matrix3by3 * matrix);
arm_status math_multVectorScalar(vector3fData * vector, float32_t scalar);
arm_status math_vectCrossProduct(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC);
arm_status math_multMatrices(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC);
arm_status math_copyMatrix(matrix3by3 * matrixA, matrix3by3 * matrixB);
arm_status math_multVectorMatrix(vector3fData * vectorA, matrix3by3 * matrix, vector3fData * vectorB);

#endif /* MATH_CUSTOM_H_ */
