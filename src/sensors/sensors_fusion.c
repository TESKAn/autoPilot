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
	Vectorf error_gps_acc = vectorf_init(0);				// Error from GPS and accelerometer data
	Vectorf error_acc_gravity = vectorf_init(0);			// Error from acceleration and gravity
	Vectorf error = vectorf_init(0);						// Cumulative error
	Vectorf omega = vectorf_init(0);						// Rotation value
	Matrixf updateMatrix;									// Update matrix
	Matrixf newMatrix;										// Place to store new DCM matrix
	float32_t fTemp = 0;
	float32_t dt = 0;

	// Check if we have valid GPS and accelerometer data
	// We need speed so do not use if below some value
	if((1 ==_GPSData.valid)&&(1 == _accData.valid)&&(param_min_airspeed < vectorf_getNorm(&_GPSData.speed3D)))
	{
		// Use GPS and accelerometer speed data to calculate DCM error in earth frame
		vectorf_crossProduct(&_accData.Speed_3D, &_GPSData.speed3D, &temporaryVector);
		// Transform to plane frame with DCM transpose
		matrix3_transposeVectorMultiply(&_fusion_DCM, &temporaryVector, &error_gps_acc);
	}
	// Check if we have valid gyro and accelerometer data and calculate error
	// Do only if speed below limit
	if((param_min_airspeed > vectorf_getNorm(&_GPSData.speed3D))&&(1 == _accData.valid))
	{
		// Store estimated gravity vector
		temporaryVector.x = _fusion_DCM.c.x;
		temporaryVector.y = _fusion_DCM.c.y;
		temporaryVector.z = _fusion_DCM.c.z;
		vectorf_crossProduct(&temporaryVector, &_accData.vector, &error_acc_gravity);
	}

	// Check if we have valid magnetometer data and calculate error

	// Check if we have valid GPS data and calculate yaw error

	// Calculate delta time
	dt = (float32_t)_gyroData.deltaTime;
	dt = dt * param_systime_toseconds;

	// If error above limit, update PID
	fTemp = vectorf_getNorm(&error);
	if(param_min_rot_error < fTemp)
	{
		// Calculate delta time in seconds
		math_PID3(&error, dt, &_gyroErrorPID);
	}

	// Remove error from gyro data
	temporaryVector.x = _gyroData.vector.x - _gyroErrorPID.x.result;
	temporaryVector.y = _gyroData.vector.y - _gyroErrorPID.y.result;
	temporaryVector.z = _gyroData.vector.z - _gyroErrorPID.z.result;

	// Calculate rotation angle
	// Is gyro value * delta time in seconds
	vectorf_scalarProduct(&_gyroData.vector, dt, &omega);

	// If rotation angle above limit, update rotation matrix
	fTemp = vectorf_getNorm(&omega);
	if(param_min_rotation < fTemp)
	{
		// Generate update matrix
		fusion_generateUpdateMatrix(&omega, &updateMatrix);
		// Multiply DCM and update matrix
		matrix3_MatrixMultiply(&updateMatrix, &_fusion_DCM, &newMatrix);

		// Renormalize and orthogonalize DCM matrix
		status = matrix3_normalizeOrthogonalizeMatrix(&newMatrix, param_dcm_max_orth_error);

		// Check if matrix is OK and copy data
		if(SUCCESS == status)
		{
			matrix3_copy(&newMatrix, &_fusion_DCM);
		}
	}
	return status;
}

/*
ErrorStatus fusion_updateRotationMatrix()
{
	ErrorStatus status;
	// All sensor readings must be updated separately

	// Update error calculation every n-th iteration


	errorUpdateInterval++;
	if(errorUpdateInterval > 10)
	{
		errorUpdateInterval = 0;
		// Update error once every second
		// If speed is below GPS limit or if GPS data is not valid
		if((ahrs_data.GPSData.speed < DEFAULT_USE_GPS_SPEED)||(ahrs_data.GPSData.dataValid == INVALID))
		{
			// Calculate gravity vector
			ahrs_data.GravityVector.vector.pData[VECT_X] = ahrs_data.AccVector.vector.pData[VECT_X];
			ahrs_data.GravityVector.vector.pData[VECT_Y] = ahrs_data.AccVector.vector.pData[VECT_Y];
			ahrs_data.GravityVector.vector.pData[VECT_Z] = ahrs_data.AccVector.vector.pData[VECT_Z];
			// Remove angular acceleration from acceleration result
			// Angular acceleration = cross product of velocity and rotation rate

			// Normalize gravity
			ahrs_normalize_vector(&(ahrs_data.GravityVector));

			// Get plane reference gravity vector
			tempVector.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rzx];
			tempVector.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Rzy];
			tempVector.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzz];

			// Calculate roll pitch error
			ahrs_vect_cross_product(&tempVector, &(ahrs_data.GravityVector), &(ahrs_data.RollPitchCorrection));
			// Scale error
			//ahrs_mult_vector_scalar(&(ahrs_data.RollPitchCorrection), ahrs_data.RollPitchCorrectionScale);

			// Add yaw error
			// Which is conveniently mag vector normalized
			// and take mag Y as yaw Z error
			tempVector.vector.pData[VECT_X] = 0;
			tempVector.vector.pData[VECT_Y] = 0;
			tempVector.vector.pData[VECT_Z] = ahrs_data.MagInEarthFrame.vector.pData[VECT_Y];
			// Just transform it to plane coordinates
			ahrs_mult_vector_matrix_transpose(&(ahrs_data.rotationMatrix), &tempVector, &(ahrs_data.YawCorrection));


			ahrs_data.Wrp = ahrs_get_vector_norm(&(ahrs_data.RollPitchCorrection)) * ahrs_data.RollPitchCorrectionScale;
			if(ahrs_data.Wrp < 0)
			{
				ahrs_data.Wrp = -ahrs_data.Wrp;
			}

			if(ahrs_data.Wrp > 1)
			{
				ahrs_data.Wrp = 1;
			}

			ahrs_data.Wy = ahrs_get_vector_norm(&(ahrs_data.YawCorrection)) * ahrs_data.YawCorrectionScale;
			if(ahrs_data.Wy < 0)
			{
				ahrs_data.Wy = -ahrs_data.Wy;
			}
			if(ahrs_data.Wy > 1)
			{
				ahrs_data.Wy = 1;
			}

			// Add correction to rollPitchCorrection vector
			ahrs_data.totalCorrectionError.vector.pData[VECT_X] = ahrs_data.RollPitchCorrection.vector.pData[VECT_X];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_X] * ahrs_data.Wy;
			ahrs_data.totalCorrectionError.vector.pData[VECT_Y] = ahrs_data.RollPitchCorrection.vector.pData[VECT_Y];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Y] * ahrs_data.Wy;
			ahrs_data.totalCorrectionError.vector.pData[VECT_Z] = ahrs_data.RollPitchCorrection.vector.pData[VECT_Z];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Z] * ahrs_data.Wy;

			// Check error change
			dT = ahrs_vector_magnitude(&(ahrs_data.totalCorrectionError));
			if(dT > ahrs_data.PIDErrorThreshold)
			{
				// Only update PID if error is > than threshold
				// Update PI error regulator
				ahrs_updateVectorPID(&(ahrs_data.PIData), &(ahrs_data.totalCorrectionError), ahrs_data.GyroVector.deltaTime);
			}
		}
		// Else use GPS data to compensate
		else
		{

		}
	}

	// Calculate change in time
	dT = (float)(data->GyroVector.deltaTime);
	dT = dT * SYSTIME_TOSECONDS;
	// Calculate change in angles
	// Calculate change in radians/sec
	dWx = data->GyroVector.vector.pData[VECT_X];
	dWy = data->GyroVector.vector.pData[VECT_Y];
	dWz = data->GyroVector.vector.pData[VECT_Z];

	// If run for first time
	if(AHRS_FIRSTRUN_PID)
	{
		// Update PID I term with initial
		ahrs_data.PIData.Ix = dWx;
		ahrs_data.PIData.Iy = dWy;
		ahrs_data.PIData.Iz = dWz;

		ahrs_data.PIData.Rx = dWx;
		ahrs_data.PIData.Ry = dWy;
		ahrs_data.PIData.Rz = dWz;

		AHRS_FIRSTRUN_PID = 0;
	}

	if(AHRS_FIRSTRUN_MATRIX)
	{
		// Update rotation matrix for initial state
		// Generate update matrix
		ahrs_generate_rotationUpdateMatrix(ahrs_data.totalCorrectionError.vector.pData[VECT_X], ahrs_data.totalCorrectionError.vector.pData[VECT_Y], ahrs_data.totalCorrectionError.vector.pData[VECT_Z], &tempMatrix);
		// Update main rot matrix
		status = ahrs_mult_matrixes(&(ahrs_data.rotationMatrix), &tempMatrix, &holdMatrix);
		// Copy new matrix
		if(status == ARM_MATH_SUCCESS)
		{
			ahrs_data.rotationMatrix.dataTime = getSystemTime();
			for(i=0; i < 9; i++)
			{
				ahrs_data.rotationMatrix.vector.pData[i] = holdMatrix.vector.pData[i];
			}
			// Normalize and orthogonalize matrix
			ahrs_normalizeOrthogonalizeMatrix(&(ahrs_data.rotationMatrix));
		}

		AHRS_FIRSTRUN_MATRIX = 0;
	}

	// Remove drift error
	dWx = dWx - data->PIData.Rx;
	dWy = dWy - data->PIData.Ry;
	dWz = dWz - data->PIData.Rz;

	// Calculate change in delta time
	dWx = dWx * dT;
	dWy = dWy * dT;
	dWz = dWz * dT;


	ahrs_data.GyroValueAdjusted.vector.pData[VECT_X] = dWx;
	ahrs_data.GyroValueAdjusted.vector.pData[VECT_Y] = dWy;
	ahrs_data.GyroValueAdjusted.vector.pData[VECT_Z] = dWz;

	// Check rotation magnitude
	dT = ahrs_vector_magnitude(&(ahrs_data.GyroValueAdjusted));
	// If rotation is larger than error, update matrix
	if(dT > DEFAULT_MIN_ROTATION_RATE)
	{
		// Generate update matrix
		ahrs_generate_rotationUpdateMatrix(dWx, dWy, dWz, &tempMatrix);
		// Update main rot matrix
		//status = ahrs_mult_matrixes(&(ahrs_data.rotationMatrix), &tempMatrix, &holdMatrix);
		status = ahrs_mult_matrixes(&tempMatrix, &(ahrs_data.rotationMatrix), &holdMatrix);
		// Copy new matrix
		if(status == SUCCESS)
		{
			data->rotationMatrix.dataTime = getSystemTime();
			for(i=0; i < 9; i++)
			{
				data->rotationMatrix.vector.pData[i] = holdMatrix.vector.pData[i];
			}
			// Normalize and orthogonalize matrix
			ahrs_normalizeOrthogonalizeMatrix(&(data->rotationMatrix));

			return SUCCESS;
		}
		else
		{
			return status;
		}
	}
	else
	{
		return SUCCESS;
	}
}
*/
