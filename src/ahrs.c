/*
 * ahrs.c
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */


#include "allinclude.h"

// Variables used
AHRSData ahrs_data;

// Temporary matrices and vectors
matrix3by3 tempMatrix;
matrix3by3 holdMatrix;
vectorData tempVector;

void initAHRSStructure(AHRSData * ahrsStructure)
{
	ahrs_vectorDataInit(&(ahrsStructure->AccVector), ROW);
	ahrs_vector3qDataInit(&(ahrsStructure->AccOffsetVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->AccScaleVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->GravityVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->GyroVector), ROW);
	ahrs_vector3qDataInit(&(ahrsStructure->GyroOffsetVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->GyroScaleVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->MagVector), ROW);
	ahrs_vector3qDataInit(&(ahrsStructure->MagOffsetVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->MagScaleVector), ROW);
	ahrs_vectorDataInit(&(ahrsStructure->RollPitchYaw), ROW);
	ahrs_matrix3by3_init(&(ahrsStructure->rotationMatrix));
	ahrsStructure->GPSData.altitude = 0;
	ahrsStructure->GPSData.dataTime = systemTime;
	ahrsStructure->GPSData.dataStartTime = systemTime;
	ahrsStructure->GPSData.dataValid = INVALID;
	ahrsStructure->GPSData.latitude = 0;
	ahrsStructure->GPSData.longitude = 0;
	ahrsStructure->GPSData.speed = 0;
	ahrsStructure->GPSData.trackAngle = 0;
	ahrs_matrix3by3_init(&(ahrsStructure->GPSReference));
	ahrs_resetRotationMatrix();
	ahrsStructure->PlaneSpeed = 0;
	ahrsStructure->PIData.Ix = 0;
	ahrsStructure->PIData.Iy = 0;
	ahrsStructure->PIData.Iz = 0;
	ahrsStructure->PIData.Kix = 0;
	ahrsStructure->PIData.Kiy = 0;
	ahrsStructure->PIData.Kiz = 0;
	ahrsStructure->PIData.Kpx = 0;
	ahrsStructure->PIData.Kpy = 0;
	ahrsStructure->PIData.Kpz = 0;
	ahrsStructure->PIData.Rx = 0;
	ahrsStructure->PIData.Ry = 0;
	ahrsStructure->PIData.Rz = 0;
	ahrsStructure->PIData.dataTime = systemTime;

	// Try to load values from SD card
	if(loadSettings() == ERROR)
	{
#ifdef DEBUG_USB
		sendUSBMessage("Error reading settings");
		sendUSBMessage("Using default values");
#endif
		// There was an error, load default values
		ahrsStructure->accRate = DEFAULT_ACC_RATE;
		ahrsStructure->gyroRate = DEFAULT_GYRO_RATE;
		ahrsStructure->magRate = DEFAULT_MAG_RATE;
	}
}

arm_status ahrs_updateAccelerationToGyro(void)
{
	// Get vectors from acceleration sensor
	// Modify with data from gyroscopes,
	return SUCCESS;
}

arm_status ahrs_updateGPSToGyro(void)
{
	// Get direction vector from GPS
	// Calculate error from GPS heading and GPS reference rotation matrix
	// Update current rotation matrix
	return SUCCESS;
}

arm_status ahrs_updateRotationMatrix(AHRSData * data)
{
	arm_status status;
	uint8_t i = 0;
	float dT = 0;
	float dWx = 0;
	float dWy = 0;
	float dWz = 0;
	float error = 0;
	// Update vectors
	updateScaledVector(&(ahrs_data.GyroVector), GYRO_X, GYRO_Y, GYRO_Z, ahrs_data.gyroRate);
	updateScaledVector(&(ahrs_data.AccVector), ACC_X, ACC_Y, ACC_Z, ahrs_data.accRate);
	updateScaledVector(&(ahrs_data.MagVector), MAG_X, MAG_Y, MAG_Z, ahrs_data.magRate);
	// Calculate gravity vector

	// Remove angular acceleration from acceleration result
	// Angular acceleration = cross product of velocity and rotation rate

	// Calculate error x
	// Update PID x
	data->PIData.Ix = data->PIData.Ix + error;
	data->PIData.Rx = (error * data->PIData.Kpx) + (data->PIData.Ix * data->PIData.Kix);

	// Calculate error y
	// Update PID y
	data->PIData.Iy = data->PIData.Iy + error;
	data->PIData.Ry = (error * data->PIData.Kpy) + (data->PIData.Iy * data->PIData.Kiy);

	// Calculate error z
	// Update PID z
	data->PIData.Iz = data->PIData.Iz + error;
	data->PIData.Rz = (error * data->PIData.Kpz) + (data->PIData.Iz * data->PIData.Kiz);

	// Calculate change in time
	dT = (float)(data->GyroVector.deltaTime) * SYSTIME_TOSECONDS;
	// Calculate change in angles
	// Calculate change in radians
	dWx = data->GyroVector.vector.pData[VECT_X] * DEG_TO_RAD;// * dT;
	dWy = data->GyroVector.vector.pData[VECT_Y] * DEG_TO_RAD;// * dT;
	dWz = data->GyroVector.vector.pData[VECT_Z] * DEG_TO_RAD;// * dT;
	// Remove drift error
	dWx = dWx + data->PIData.Rx;
	dWy = dWy + data->PIData.Ry;
	dWz = dWz + data->PIData.Rz;
	// Calculate change in delta time
	dWx = dWx * dT;
	dWy = dWy * dT;
	dWz = dWz * dT;
	// Update vector
	ahrs_vectorUpdate(&tempVector, dWx, dWy, dWz);
	// Generate update matrix
	ahrs_generate_rotationUpdateMatrix(&tempVector, &tempMatrix);
	// Update main rot matrix
	status = ahrs_mult_matrixes(&(data->rotationMatrix), &tempMatrix, &holdMatrix);
	// Copy new matrix
	if(status == ARM_MATH_SUCCESS)
	{
		data->rotationMatrix.dataTime = systemTime;
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
		return ERROR;
	}
}

void ahrs_resetRotationMatrix(void)
{
	ahrs_data.rotationMatrix.vector.pData[0] = 1;
	ahrs_data.rotationMatrix.vector.pData[1] = 0;
	ahrs_data.rotationMatrix.vector.pData[2] = 0;
	ahrs_data.rotationMatrix.vector.pData[3] = 0;
	ahrs_data.rotationMatrix.vector.pData[4] = 1;
	ahrs_data.rotationMatrix.vector.pData[5] = 0;
	ahrs_data.rotationMatrix.vector.pData[6] = 0;
	ahrs_data.rotationMatrix.vector.pData[7] = 0;
	ahrs_data.rotationMatrix.vector.pData[8] = 1;
}

/// Make matrix orthogonal and normalized
void ahrs_normalizeOrthogonalizeMatrix(matrix3by3 * rotMatrix)
{
	float32_t error = 0;
	// Calculate error = X.Y
	error = rotMatrix->vector.pData[Rxx] * rotMatrix->vector.pData[Ryx];
	error = error + rotMatrix->vector.pData[Rxy] * rotMatrix->vector.pData[Ryy];
	error = error + rotMatrix->vector.pData[Rxz] * rotMatrix->vector.pData[Ryz];
	// Add half error to X, half to Y
	error = error / 2;
	// Add to X
	rotMatrix->vector.pData[Rxx] = rotMatrix->vector.pData[Rxx] - error * rotMatrix->vector.pData[Ryx];
	rotMatrix->vector.pData[Rxy] = rotMatrix->vector.pData[Rxy] - error * rotMatrix->vector.pData[Ryy];
	rotMatrix->vector.pData[Rxz] = rotMatrix->vector.pData[Rxz] - error * rotMatrix->vector.pData[Ryz];
	// Add to Y
	rotMatrix->vector.pData[Ryx] = rotMatrix->vector.pData[Ryx] - error * rotMatrix->vector.pData[Rxx];
	rotMatrix->vector.pData[Ryy] = rotMatrix->vector.pData[Ryy] - error * rotMatrix->vector.pData[Rxy];
	rotMatrix->vector.pData[Ryz] = rotMatrix->vector.pData[Ryz] - error * rotMatrix->vector.pData[Rxz];
	// Normalize matrix X and Y
	// Calculate scalar X.X
	error = (rotMatrix->vector.pData[Rxx] * rotMatrix->vector.pData[Rxx])+(rotMatrix->vector.pData[Rxy] * rotMatrix->vector.pData[Rxy])+(rotMatrix->vector.pData[Rxz] * rotMatrix->vector.pData[Rxz]);
	// Calculate 3 - scalar
	error = 3 - error;
	// Calculate one half
	error = error / 2;
	// Multiply with X
	rotMatrix->vector.pData[Rxx] = rotMatrix->vector.pData[Rxx] * error;
	rotMatrix->vector.pData[Rxy] = rotMatrix->vector.pData[Rxy] * error;
	rotMatrix->vector.pData[Rxz] = rotMatrix->vector.pData[Rxz] * error;
	// Calculate scalar Y.Y
	error = (rotMatrix->vector.pData[Ryx] * rotMatrix->vector.pData[Ryx])+(rotMatrix->vector.pData[Ryy] * rotMatrix->vector.pData[Ryy])+(rotMatrix->vector.pData[Ryz] * rotMatrix->vector.pData[Ryz]);
	// Calculate 3 - scalar
	error = 3 - error;
	// Calculate one half
	error = error / 2;
	// Multiply with Y
	rotMatrix->vector.pData[Ryx] = rotMatrix->vector.pData[Ryx] * error;
	rotMatrix->vector.pData[Ryy] = rotMatrix->vector.pData[Ryy] * error;
	rotMatrix->vector.pData[Ryz] = rotMatrix->vector.pData[Ryz] * error;
	// Calculate Z as cross product of X and Y
	rotMatrix->vector.pData[Rzx] = (rotMatrix->vector.pData[Rxy]*rotMatrix->vector.pData[Ryz]) - (rotMatrix->vector.pData[Rxz]*rotMatrix->vector.pData[Ryy]);
	rotMatrix->vector.pData[Rzy] = (rotMatrix->vector.pData[Rxz]*rotMatrix->vector.pData[Ryx]) - (rotMatrix->vector.pData[Rxx]*rotMatrix->vector.pData[Ryz]);
	rotMatrix->vector.pData[Rzz] = (rotMatrix->vector.pData[Rxx]*rotMatrix->vector.pData[Ryy]) - (rotMatrix->vector.pData[Rxy]*rotMatrix->vector.pData[Ryx]);
	// Normalize matrix Z
	// Calculate scalar Z.Z
	error = (rotMatrix->vector.pData[Rzx] * rotMatrix->vector.pData[Rzx])+(rotMatrix->vector.pData[Rzy] * rotMatrix->vector.pData[Rzy])+(rotMatrix->vector.pData[Rzz] * rotMatrix->vector.pData[Rzz]);
	// Calculate 3 - scalar
	error = 3 - error;
	// Calculate one half
	error = error / 2;
	// Multiply with Z
	rotMatrix->vector.pData[Rzx] = rotMatrix->vector.pData[Rzx] * error;
	rotMatrix->vector.pData[Rzy] = rotMatrix->vector.pData[Rzy] * error;
	rotMatrix->vector.pData[Rzz] = rotMatrix->vector.pData[Rzz] * error;
}

void ahrs_getAngles(matrix3by3 * rotMatrix, vector3f *vector)
{
	// Store angles
	vector->pData[VECT_X] = -asin(rotMatrix->vector.pData[Rzx]) * 1000;
	vector->pData[VECT_Y] = atan2(rotMatrix->vector.pData[Rzy],  rotMatrix->vector.pData[Rzz]) * 1000;
	vector->pData[VECT_Z] = atan2(rotMatrix->vector.pData[Ryx],  rotMatrix->vector.pData[Rxx]) * 1000;
}
