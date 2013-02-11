/*
 * ahrs.h
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */

#ifndef AHRS_H_
#define AHRS_H_


// Structure for 3 floating point PI regulators
typedef struct
{
	uint32_t dataTime;
	// Proportional gain
	float32_t Kpx;
	float32_t Kpy;
	float32_t Kpz;
	// Proportional variables
	float32_t Px;
	float32_t Py;
	float32_t Pz;
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
	// Integral max value
	float32_t maxIx;
	float32_t maxIy;
	float32_t maxIz;
	// Integral min value
	float32_t minIx;
	float32_t minIy;
	float32_t minIz;
	// Error min/max value
	float32_t eMin;
	float32_t eMax;
	// Result min/max value
	float32_t rMin;
	float32_t rMax;

}__attribute__((aligned(4),packed)) PI3Data;

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

// Altitude
typedef struct
{
	uint32_t dataTime;			// Time at which data was taken
	uint32_t deltaTime;			// Time that has passed between two samples
	float currentAltitude;		// Altitude in meters
	float lastAltitude;			// Previous altitude value
	float verticalSpeed;		// Vertical speed as (currentAltitude - lastAltitude) / deltaTime
	float verticalAcceleration;	// Vertical acceleration as verticalSpeed / deltaTime
}__attribute__((aligned(4),packed)) AltitudeData;

typedef struct
{
	// Rotation quaternion
	quaternion Q;
	// Rotation matrix
	matrix3by3 rotationMatrix;
	// Acceleration vector
	vector3fData AccVector;
	// Acceleration offset vector
	vector3qData AccOffsetVector;
	// Acceleration scale vector
	vector3fData AccScaleVector;
	// Gravity vector
	vector3fData GravityVector;
	// Gyroscope vector
	vector3fData GyroVector;
	// Gyroscope offsets
	vector3qData GyroOffsetVector;
	// Gyroscope scale vector
	vector3fData GyroScaleVector;
	// Magnetometer vector
	vector3fData MagVector;
	// Magnetometer offset vector
	vector3fData MagOffsetVector;
	// Magnetometer scale vector
	vector3fData MagScaleVector;
	// Magnetometer rotation and scale matrix
	matrix3by3 magRotationMatrix;
	// Pitch, roll, yaw angles in rad
	vector3fData RollPitchYaw;
	// GPS data
	GPSTypeDef GPSData;
	// GPS reference rotation matrix
	matrix3by3 GPSReference;
	// Altitude data
	AltitudeData Altitude;
	// Speed data in m/sec
	float32_t PlaneSpeed;
	// Roll Pitch correction vector
	vector3fData RollPitchCorrection;
	// Yaw correction vector
	vector3fData YawCorrection;
	// PI data
	PI3Data PIData;
	// Scale values
	float32_t gyroRate;
	float32_t accRate;
	float32_t magRate;
	float32_t RollPitchCorrectionScale;
	float32_t YawCorrectionScale;
	// Mag and X vectors at t-1
	vector3fData magp;
	vector3fData xp;
	// Error calculated from magnetometer
	vector3fData magCorrectionError;
	// Total correction vector
	vector3fData totalCorrectionError;

}__attribute__((aligned(4),packed)) AHRSData;

extern AHRSData ahrs_data;


// Temporary matrices and vectors
extern matrix3by3 tempMatrix;
extern matrix3by3 holdMatrix;
extern vector3fData tempVector;
extern vector3fData tempVector1;
extern vector3fData tempVector2;

// Function exports - ahrs.h
void initAHRSStructure(AHRSData * ahrsStructure);
void ahrs_update_altitude(void);
arm_status ahrs_updateAccelerationToGyro(void);
arm_status ahrs_updateGPSToGyro(void);
arm_status ahrs_updateRotationMatrix(AHRSData * data);
ErrorStatus ahrs_initQuaternion(void);
ErrorStatus ahrs_updateQuaternion(void);
void ahrs_resetPID(PI3Data* PID);
void ahrs_resetRotationMatrix(void);
void ahrs_resetQuaternion(void);
void ahrs_normalizeOrthogonalizeMatrix(matrix3by3 * rotMatrix);
void ahrs_getAngles(matrix3by3 * rotMatrix, vector3f *vector);

// Function exports - ahrs_math.h
arm_status ahrs_updateVectorPID(PI3Data* PID, vector3fData * errorVector, float32_t deltaT);
float32_t ahrs_limitFloat(float32_t number, float32_t max, float32_t min);
void updateScaledVector(vector3fData * vector, uint16_t x, uint16_t y, uint16_t z, float rate);
void ahrs_vector3fDataInit(vector3fData * vector, VectType type);
void ahrs_vector3qDataInit(vector3qData * vector, VectType type);
void ahrs_vectorUpdate(vector3fData * vector, float32_t i, float32_t j, float32_t k);
void ahrs_matrix3by3_init(matrix3by3 * matrix);
void ahrs_generate_rotationMatrix(matrix3by3 * matrix, float roll, float pitch, float yaw);
void ahrs_generate_rotationUpdateMatrix(float32_t x, float32_t y, float32_t z, matrix3by3 * matrix);
arm_status ahrs_vector_substract(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC);
arm_status ahrs_mult_vector_scalar(vector3fData * vectorA, float32_t scalar);
arm_status ahrs_vect_dot_product(vector3fData * vectorA, vector3fData * vectorB, float32_t * scalarC);
arm_status ahrs_vect_cross_product(vector3fData * vectorA, vector3fData * vectorB, vector3fData * vectorC);
arm_status ahrs_mult_matrixes(matrix3by3 * matrixA, matrix3by3 * matrixB, matrix3by3 * matrixC);
arm_status ahrs_mult_vector_matrix(matrix3by3 * matrixA, vector3fData * vectorA, vector3fData * vectorB);
arm_status ahrs_matrix_transponse(matrix3by3 * matrixA, matrix3by3 * matrixB);
arm_status ahrs_normalize_vector_taylor(vector3fData * vectorA);
arm_status ahrs_normalize_vector(vector3fData * vectorA);
arm_status ahrs_copy_vector(vector3fData * vectorA, vector3fData * vectorB);

#endif /* AHRS_H_ */
