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

typedef struct
{
	uint32_t dataTime;
	VectType type;
	vector3f vector;
	float32_t vectorData[3];
}vectorData;
// Define vector components
#define VECT_X		0
#define VECT_Y		1
#define VECT_Z		2


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

// Function exports
void updateScaledVector(vectorData * vector, uint16_t x, uint16_t y, uint16_t z, float rate);
void ahrs_vectorDataInit(vectorData * vector);
void ahrs_vectorUpdate(vectorData * vector, float32_t i, float32_t j, float32_t k);
void ahrs_vector_init(vector3f * S, float32_t * pData, VectType type);

#endif /* AHRS_H_ */
