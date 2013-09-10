/*
 * sensors_fusion.c
 *
 *  Created on: Aug 22, 2013
 *      Author: Jure
 */

#include "stm32f4xx.h"
#include "arm_math.h"
#include "functions.h"
#include "parameters.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"
#include "math/myMath_matrix3.h"
#include "math/myMath_pid.h"
#include "sensors_typedefs.h"
#include "gyro.h"
#include "accelerometer.h"
#include "mag.h"
#include "airSpeed.h"
#include "gps.h"
#include "sensors_fusion.h"

Matrixf _fusion_DCM;

ErrorStatus fusion_generateUpdateMatrix(Vectorf * omega, Matrixf * updateMatrix)
{
	ErrorStatus status = ERROR;

	// First create identity matrix
	matrix3_init(1, updateMatrix);
	// Populate elements of matrix
	updateMatrix->a.y = -omega->z;
	updateMatrix->a.z = omega->y;

	updateMatrix->b.x = omega->z;
	updateMatrix->b.z = -omega->x;

	updateMatrix->c.x = -omega->y;
	updateMatrix->c.y = omega->x;

	status = SUCCESS;
	return status;
}

ErrorStatus fusion_updateRotationMatrix()
{
	ErrorStatus status;
	status = SUCCESS;
	Vectorf temporaryVector = vectorf_init(0);
	Vectorf temporaryVector1 = vectorf_init(0);
	Vectorf temporaryVector2 = vectorf_init(0);
	Vectorf error_gps_acc = vectorf_init(0);				// Error from GPS and accelerometer data
	Vectorf error_acc_gravity = vectorf_init(0);			// Error from acceleration and gravity
	Vectorf error_yaw_gps = vectorf_init(0);				// Error calculated from GPS data
	Vectorf error_yaw_mag = vectorf_init(0);				// Error calculated from magnetometer data
	Vectorf error = vectorf_init(0);						// Cumulative error
	Vectorf omega = vectorf_init(0);						// Rotation value
	Matrixf updateMatrix;									// Update matrix
	Matrixf newMatrix;										// Place to store new DCM matrix
	float32_t fTemp = 0;
	float32_t dt = 0;
	float32_t GPSSpeed = 0;

	// Calculate GPS speed
	GPSSpeed = vectorf_getNorm(&_GPSData.speed3D);

	// Check if we have valid GPS and accelerometer data
	// We need speed so do not use if below some value
	if((1 ==_GPSData.valid)&&(1 == _accData.valid)&&(param_min_airspeed < GPSSpeed))
	{
		// Use GPS and accelerometer speed data to calculate DCM error in earth frame
		// Calculate 1/Dt
		fTemp = 1 / _accData.speed_3D_dt;
		// Calculate average acceleration of accelerometer
		status = vectorf_scalarProduct(&_accData.Speed_3D, fTemp, &temporaryVector);
		// Calculate [0,0,1] - average acceleration of GPS
		status = vectorf_scalarProduct(&_GPSData.speed3D, -fTemp, &temporaryVector2);
		temporaryVector2.z = temporaryVector2.z + 1;
		// Calculate error
		status = vectorf_crossProduct(&temporaryVector, &temporaryVector2, &error_gps_acc);
		// Make yaw error z = 0 - we will calculate that error from heading
		error_gps_acc.z = 0;

		// GPS data is OK so calculate yaw error from GPS data -> temporaryVector
		// Get plane X vector in earth coordinates - that's where we are pointing
		// Take plane X in earth coordinates - that's column 1 (i think)
		// Set Z to zero - we don't need it
		temporaryVector.z = 0;
		// Normalize vector to length 1
		status = vectorf_normalize(&temporaryVector);

		// Calculate GPS heading vector from GPS angle -> temporaryVector1
		// x = cos(x)
		// y = sin(y)
		// Normalize this vector to length 1
		status = vectorf_normalize(&temporaryVector1);

		// Error is cross product between these two vectors
		status = vectorf_crossProduct(&temporaryVector, &temporaryVector1, &temporaryVector2);
		// We only need the Z component so copy x and y from total error
		temporaryVector2.x = error_gps_acc.x;
		temporaryVector2.y = error_gps_acc.y;

		// Transform to plane frame with DCM transpose
		status = matrix3_transposeVectorMultiply(&_fusion_DCM, &temporaryVector2, &error_gps_acc);

		// Check if all was calculated OK
		if(ERROR == status)
		{
			// There was error, so reset error vector
			error_gps_acc = vectorf_init(0);
		}

	}
	// Check if we have valid gyro and accelerometer data and calculate error
	// Do only if speed below limit
	if((param_min_airspeed > GPSSpeed)&&(1 == _accData.valid))
	{
		// Store estimated gravity vector
		temporaryVector.x = _fusion_DCM.c.x;
		temporaryVector.y = _fusion_DCM.c.y;
		temporaryVector.z = _fusion_DCM.c.z;
		status = vectorf_crossProduct(&temporaryVector, &_accData.vector, &error_acc_gravity);
		// Check if there was calculation error
		// If yes, set error to 0
		if(ERROR == status)
		{
			error_acc_gravity = vectorf_init(0);
		}
	}

	// Check if we have valid magnetometer data and calculate error

	// Check if we have valid GPS data and calculate yaw error


	//error_yaw_gps

	// Calculate delta time
	dt = (float32_t)_gyroData.deltaTime;
	dt = dt * param_systime_toseconds;

	// If error above limit, update PID
	fTemp = vectorf_getNorm(&error);
	if(param_min_rot_error < fTemp)
	{
		// Calculate delta time in seconds
		status = math_PID3(&error, dt, &_gyroErrorPID);
	}

	// Remove error from gyro data
	// _gyroError is not vector
	temporaryVector.x = _gyroData.vector.x - _gyroErrorPID.x.result;
	temporaryVector.y = _gyroData.vector.y - _gyroErrorPID.y.result;
	temporaryVector.z = _gyroData.vector.z - _gyroErrorPID.z.result;

	// Calculate rotation angle
	// Is gyro value * delta time in seconds
	status = vectorf_scalarProduct(&_gyroData.vector, dt, &omega);

	// If rotation angle above limit, update rotation matrix
	fTemp = vectorf_getNorm(&omega);
	if(param_min_rotation < fTemp)
	{
		// Generate update matrix
		status = fusion_generateUpdateMatrix(&omega, &updateMatrix);
		// Multiply DCM and update matrix
		status = matrix3_MatrixMultiply(&updateMatrix, &_fusion_DCM, &newMatrix);

		// Renormalize and orthogonalize DCM matrix
		status = matrix3_normalizeOrthogonalizeMatrix(&newMatrix, param_dcm_max_orth_error);

		// Check if matrix is OK and copy data
		if(SUCCESS == status)
		{
			matrix3_copy(&newMatrix, &_fusion_DCM);
		}
	}

	// Update speed integration
	// Transform accelerometer data to earth frame
	status = matrix3_vectorMultiply(&_fusion_DCM, &_accData.vector, &temporaryVector);
	// Calculate 3D speed gained, acceleration * time
	// Do not worry about gravity acceleration, we remove that in GPS error update
	status = vectorf_scalarProduct(&temporaryVector, dt, &temporaryVector);
	// Integrate speed and time
	status = vectorf_add(&temporaryVector, &_accData.Speed_3D, &_accData.Speed_3D);
	// Time integration can have problems after 2,77 hours because of 32 bit float representation
	_accData.speed_3D_dt = _accData.speed_3D_dt + dt;

	return status;
}

