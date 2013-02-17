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
vector3fData tempVector;
vector3fData tempVector1;
vector3fData tempVector2;
volatile quaternion Qtemp;

void initAHRSStructure(AHRSData * ahrsStructure)
{
	// Init quaternion to 0 rotation
	ahrsStructure->Q.w = 1;
	ahrsStructure->Q.x = 0;
	ahrsStructure->Q.y = 0;
	ahrsStructure->Q.z = 0;
	ahrsStructure->Q.dataTime = systemTime;
	ahrsStructure->Q.fDataTime = getFTime();
	ahrsStructure->Q.fDeltaTime = 0;
	math_vector3fDataInit(&(ahrsStructure->AccVector), ROW);
	math_vector3qDataInit(&(ahrsStructure->AccOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->AccScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GravityVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GyroVector), ROW);
	math_vector3qDataInit(&(ahrsStructure->GyroOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GyroScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->RollPitchYaw), ROW);
	math_matrix3by3Init(&(ahrsStructure->rotationMatrix), MATH_YES);
	math_matrix3by3Init(&(ahrsStructure->magRotationMatrix), MATH_YES);
	ahrsStructure->GPSData.altitude = 0;
	ahrsStructure->GPSData.dataTime = systemTime;
	ahrsStructure->GPSData.dataStartTime = systemTime;
	ahrsStructure->GPSData.dataValid = INVALID;
	ahrsStructure->GPSData.latitude = 0;
	ahrsStructure->GPSData.longitude = 0;
	ahrsStructure->GPSData.speed = 0;
	ahrsStructure->GPSData.trackAngle = 0;
	math_matrix3by3Init(&(ahrsStructure->GPSReference), MATH_NO);
	ahrsStructure->Altitude.currentAltitude = 0;
	ahrsStructure->Altitude.dataTime = systemTime;
	ahrsStructure->Altitude.deltaTime = 0;
	ahrsStructure->Altitude.lastAltitude = 0;
	ahrsStructure->Altitude.verticalAcceleration = 0;
	ahrsStructure->Altitude.verticalSpeed = 0;
	ahrs_resetRotationMatrix();
	ahrsStructure->PlaneSpeed = 0;
	ahrsStructure->PIData.Ix = 0;
	ahrsStructure->PIData.Iy = 0;
	ahrsStructure->PIData.Iz = 0;
	ahrsStructure->PIData.Kix = DEFAULT_KI;
	ahrsStructure->PIData.Kiy = DEFAULT_KI;
	ahrsStructure->PIData.Kiz = DEFAULT_KI;
	ahrsStructure->PIData.Px = 0;
	ahrsStructure->PIData.Py = 0;
	ahrsStructure->PIData.Pz = 0;
	ahrsStructure->PIData.Kpx = DEFAULT_KP;
	ahrsStructure->PIData.Kpy = DEFAULT_KP;
	ahrsStructure->PIData.Kpz = DEFAULT_KP;
	ahrsStructure->PIData.Rx = 0;
	ahrsStructure->PIData.Ry = 0;
	ahrsStructure->PIData.Rz = 0;
	ahrsStructure->PIData.maxIx = DEFAULT_MAXI;
	ahrsStructure->PIData.maxIy = DEFAULT_MAXI;
	ahrsStructure->PIData.maxIz = DEFAULT_MAXI;
	ahrsStructure->PIData.minIx = DEFAULT_MINI;
	ahrsStructure->PIData.minIy = DEFAULT_MINI;
	ahrsStructure->PIData.minIz = DEFAULT_MINI;
	ahrsStructure->PIData.rMax = DEFAULT_RMAX;
	ahrsStructure->PIData.rMin = DEFAULT_RMIN;
	ahrsStructure->PIData.dataTime = systemTime;
	math_vector3fDataInit(&(ahrsStructure->RollPitchCorrection), ROW);
	math_vector3fDataInit(&(ahrsStructure->YawCorrection), ROW);
	ahrsStructure->RollPitchCorrectionScale = DEFAULT_ROLLPITCHCORRECTIONSCALE;
	ahrsStructure->YawCorrectionScale = DEFAULT_YAWCORRECTIONSCALE;
	math_vector3fDataInit(&(ahrsStructure->magp), ROW);
	math_vector3fDataInit(&(ahrsStructure->xp), ROW);
	math_vector3fDataInit(&(ahrsStructure->magCorrectionError), ROW);
	math_vector3fDataInit(&(ahrsStructure->totalCorrectionError), ROW);
	// Store offsets for mag
	ahrsStructure->MagOffsetVector.vector.pData[VECT_X] = HARDIRON_DEFAULT_X;
	ahrsStructure->MagOffsetVector.vector.pData[VECT_Y] = HARDIRON_DEFAULT_Y;
	ahrsStructure->MagOffsetVector.vector.pData[VECT_Z] = HARDIRON_DEFAULT_Z;
	// Store transformation matrix for mag
	ahrsStructure->magRotationMatrix.vector.pData[Rxx] = SOFTMAG_DEFAULT_RXX;
	ahrsStructure->magRotationMatrix.vector.pData[Ryx] = SOFTMAG_DEFAULT_RYX;
	ahrsStructure->magRotationMatrix.vector.pData[Rzx] = SOFTMAG_DEFAULT_RZX;
	ahrsStructure->magRotationMatrix.vector.pData[Rxy] = SOFTMAG_DEFAULT_RXY;
	ahrsStructure->magRotationMatrix.vector.pData[Ryy] = SOFTMAG_DEFAULT_RYY;
	ahrsStructure->magRotationMatrix.vector.pData[Rzy] = SOFTMAG_DEFAULT_RZY;
	ahrsStructure->magRotationMatrix.vector.pData[Rxz] = SOFTMAG_DEFAULT_RXZ;
	ahrsStructure->magRotationMatrix.vector.pData[Ryz] = SOFTMAG_DEFAULT_RYZ;
	ahrsStructure->magRotationMatrix.vector.pData[Rzz] = SOFTMAG_DEFAULT_RZZ;

	// Min Max error values
	ahrsStructure->PIData.eMax = DEFAULT_MAXERR;
	ahrsStructure->PIData.eMin = DEFAULT_MINERR;

	ahrsStructure->Heading = 0;
	// Try to load values from SD card
	if(loadSettings() == ERROR)
	{
#ifdef DEBUG_USB
		sendUSBMessage("Error reading settings");
		sendUSBMessage("Using default values");
#endif
	}

	ahrsStructure->PIData.Kix = DEFAULT_KI;
	ahrsStructure->PIData.Kiy = DEFAULT_KI;
	ahrsStructure->PIData.Kiz = DEFAULT_KI;
	ahrsStructure->PIData.Px = 0;
	ahrsStructure->PIData.Py = 0;
	ahrsStructure->PIData.Pz = 0;
	ahrsStructure->PIData.Kpx = DEFAULT_KP;
	ahrsStructure->PIData.Kpy = DEFAULT_KP;
	ahrsStructure->PIData.Kpz = DEFAULT_KP;
	ahrsStructure->PIData.Rx = 0;
	ahrsStructure->PIData.Ry = 0;
	ahrsStructure->PIData.Rz = 0;
	ahrsStructure->PIData.maxIx = DEFAULT_MAXI;
	ahrsStructure->PIData.maxIy = DEFAULT_MAXI;
	ahrsStructure->PIData.maxIz = DEFAULT_MAXI;
	ahrsStructure->PIData.minIx = DEFAULT_MINI;
	ahrsStructure->PIData.minIy = DEFAULT_MINI;
	ahrsStructure->PIData.minIz = DEFAULT_MINI;
	ahrsStructure->PIData.rMax = DEFAULT_RMAX;
	ahrsStructure->PIData.rMin = DEFAULT_RMIN;
}

void ahrs_updateMagReading(void)
{
	// Store previous vector
	ahrs_copy_vector(&(ahrs_data.MagVector), &(ahrs_data.magp));
	// Store data to vector and remove offset
	tempVector.vector.pData[VECT_X] = (float32_t)((int16_t)MAG_X);
	tempVector.vector.pData[VECT_X] = tempVector.vector.pData[VECT_X] - ahrs_data.MagOffsetVector.vector.pData[VECT_X];

	tempVector.vector.pData[VECT_Y] = (float32_t)((int16_t)MAG_Y);
	tempVector.vector.pData[VECT_Y] = tempVector.vector.pData[VECT_Y] - ahrs_data.MagOffsetVector.vector.pData[VECT_Y];

	tempVector.vector.pData[VECT_Z] = (float32_t)((int16_t)MAG_Z);
	tempVector.vector.pData[VECT_Z] = tempVector.vector.pData[VECT_Z] - ahrs_data.MagOffsetVector.vector.pData[VECT_Z];
	// Transform
	ahrs_mult_vector_matrix(&(ahrs_data.magRotationMatrix), &tempVector, &(ahrs_data.MagVector));
	// Normalize
	ahrs_normalize_vector(&(ahrs_data.MagVector));
}

void ahrs_update_altitude(void)
{
	ahrs_data.Altitude.currentAltitude = (float)((int16_t)BARO) + (float)BARO_FRAC/10;
	ahrs_data.Altitude.deltaTime = systemTime - ahrs_data.Altitude.dataTime;
	ahrs_data.Altitude.dataTime = systemTime;
	ahrs_data.Altitude.verticalSpeed = (ahrs_data.Altitude.lastAltitude - ahrs_data.Altitude.currentAltitude) / (ahrs_data.Altitude.deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.verticalAcceleration = ahrs_data.Altitude.verticalSpeed / (ahrs_data.Altitude.deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.lastAltitude = ahrs_data.Altitude.currentAltitude;
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
	// Update vectors
	updateScaledVector(&(ahrs_data.GyroVector), GYRO_X, GYRO_Y, GYRO_Z, ahrs_data.gyroRate * DEG_TO_RAD);
	updateScaledVector(&(ahrs_data.AccVector), ACC_X, ACC_Y, ACC_Z, ahrs_data.accRate);
	updateScaledVector(&(ahrs_data.MagVector), MAG_X, MAG_Y, MAG_Z, ahrs_data.magRate);
	// Update altitude reading
	ahrs_update_altitude();
	// Update mag reading
	ahrs_updateMagReading();
	// Inverse if necessary
	ahrs_data.AccVector.vector.pData[VECT_X] = -ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.AccVector.vector.pData[VECT_Y] = -ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.AccVector.vector.pData[VECT_Z] = -ahrs_data.AccVector.vector.pData[VECT_Z];


	// Calculate gravity vector
	ahrs_data.GravityVector.vector.pData[VECT_X] = ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.GravityVector.vector.pData[VECT_Y] = ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.GravityVector.vector.pData[VECT_Z] = ahrs_data.AccVector.vector.pData[VECT_Z];
	// Remove angular acceleration from acceleration result
	// Angular acceleration = cross product of velocity and rotation rate

	// Get plane reference gravity vector
	tempVector.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rzx];
	tempVector.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Rzy];
	tempVector.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzz];


	// Calculate roll pitch error
	ahrs_vect_cross_product(&tempVector, &(ahrs_data.GravityVector), &(ahrs_data.RollPitchCorrection));
	// Scale error
	ahrs_mult_vector_scalar(&(ahrs_data.RollPitchCorrection), ahrs_data.RollPitchCorrectionScale);

	// Add yaw error


	// Update PI error regulator
	ahrs_updateVectorPID(&(ahrs_data.PIData), &(ahrs_data.RollPitchCorrection), ahrs_data.GyroVector.fDeltaTime);

	// Calculate change in time
	dT = (float)(data->GyroVector.deltaTime) * SYSTIME_TOSECONDS;
	// Calculate change in angles
	// Calculate change in radians/sec
	dWx = data->GyroVector.vector.pData[VECT_X];// * dT;
	dWy = data->GyroVector.vector.pData[VECT_Y];// * dT;
	dWz = data->GyroVector.vector.pData[VECT_Z];// * dT;
	// Remove drift error
	dWx = dWx - data->PIData.Rx;
	dWy = dWy - data->PIData.Ry;
	dWz = dWz - data->PIData.Rz;

	// Calculate change in delta time
	dWx = dWx * dT;
	dWy = dWy * dT;
	dWz = dWz * dT;
	// Generate update matrix
	ahrs_generate_rotationUpdateMatrix(dWx, dWy, dWz, &tempMatrix);
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

// Initialize quaternion yaw with data from magnetometer
ErrorStatus ahrs_initQuaternion(void)
{

	//asinf
	return SUCCESS;
}


ErrorStatus ahrs_updateQuaternion(void)
{
	float32_t delta = 0;

	float32_t xx = 0;
	float32_t yy = 0;
	float32_t zz = 0;
	float32_t xy = 0;
	float32_t xz = 0;
	float32_t yz = 0;
	float32_t wx = 0;
	float32_t wy = 0;
	float32_t wz = 0;
	float32_t dWx = 0;
	float32_t dWy = 0;
	float32_t dWz = 0;
	float32_t dT = 0;

	// Update vectors
	updateScaledVector(&(ahrs_data.GyroVector), GYRO_X, GYRO_Y, GYRO_Z, ahrs_data.gyroRate * DEG_TO_RAD);
	updateScaledVector(&(ahrs_data.AccVector), ACC_X, ACC_Y, ACC_Z, ahrs_data.accRate);
	//updateScaledVector(&(ahrs_data.MagVector), MAG_X, MAG_Y, MAG_Z, ahrs_data.magRate);
	// Update altitude reading
	ahrs_update_altitude();
	// Update mag reading
	ahrs_updateMagReading();
	// Inverse if necessary
	ahrs_data.AccVector.vector.pData[VECT_X] = -ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.AccVector.vector.pData[VECT_Y] = -ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.AccVector.vector.pData[VECT_Z] = -ahrs_data.AccVector.vector.pData[VECT_Z];

	// Calculate change in radians/sec
	dWx = ahrs_data.GyroVector.vector.pData[VECT_X];// * dT;
	dWy = ahrs_data.GyroVector.vector.pData[VECT_Y];// * dT;
	dWz = ahrs_data.GyroVector.vector.pData[VECT_Z];// * dT;

	// Remove drift error

	dWx = dWx - ahrs_data.PIData.Rx;
	dWy = dWy - ahrs_data.PIData.Ry;
	dWz = dWz - ahrs_data.PIData.Rz;


	// Store current quaternion
	Qtemp.w = ahrs_data.Q.w;
	Qtemp.x = ahrs_data.Q.x;
	Qtemp.y = ahrs_data.Q.y;
	Qtemp.z = ahrs_data.Q.z;

	xx = ahrs_data.Q.x;
	yy = ahrs_data.Q.y;
	zz = ahrs_data.Q.z;

    dT = (float)(ahrs_data.GyroVector.deltaTime) * SYSTIME_TOSECONDS;

    Qtemp.w = Qtemp.w + (-Qtemp.x*dWx-Qtemp.y*dWy-Qtemp.z*dWz)*(0.5 * dT);
    Qtemp.x = xx + (Qtemp.w * dWx + yy * dWz - zz * dWy)*(0.5 * dT);
    Qtemp.y = yy + (Qtemp.w * dWy - xx * dWz + zz * dWx)*(0.5 * dT);
    Qtemp.z = zz + (Qtemp.w * dWz + xx * dWy - yy * dWx)*(0.5 * dT);
    ahrs_data.Q.w = Qtemp.w;
    ahrs_data.Q.x = Qtemp.x;
    ahrs_data.Q.y = Qtemp.y;
    ahrs_data.Q.z = Qtemp.z;


	// Generate new rotation matrix
	// Calculate intermittent products
	xx = 2 * ahrs_data.Q.x * ahrs_data.Q.x;
	yy = 2 * ahrs_data.Q.y * ahrs_data.Q.y;
	zz = 2 * ahrs_data.Q.z * ahrs_data.Q.z;
	xy = 2 * ahrs_data.Q.x * ahrs_data.Q.y;
	xz = 2 * ahrs_data.Q.x * ahrs_data.Q.z;
	yz = 2 * ahrs_data.Q.y * ahrs_data.Q.z;
	wx = 2 * ahrs_data.Q.w * ahrs_data.Q.x;
	wy = 2 * ahrs_data.Q.w * ahrs_data.Q.y;
	wz = 2 * ahrs_data.Q.w * ahrs_data.Q.z;

	ahrs_data.rotationMatrix.vector.pData[Rxx] = 1 - yy - zz;
	ahrs_data.rotationMatrix.vector.pData[Ryx] = xy - wz;
	ahrs_data.rotationMatrix.vector.pData[Rzx] = xz + wy;
	ahrs_data.rotationMatrix.vector.pData[Rxy] = xy + wz;
	ahrs_data.rotationMatrix.vector.pData[Ryy] = 1 - xx - zz;
	ahrs_data.rotationMatrix.vector.pData[Rzy] = yz - wx;
	ahrs_data.rotationMatrix.vector.pData[Rzx] = xz - wy;
	ahrs_data.rotationMatrix.vector.pData[Rzy] = yz + wx;
	ahrs_data.rotationMatrix.vector.pData[Rzz] = 1 - xx - yy;


	// Calculate gravity vector
	ahrs_data.GravityVector.vector.pData[VECT_X] = ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.GravityVector.vector.pData[VECT_Y] = ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.GravityVector.vector.pData[VECT_Z] = ahrs_data.AccVector.vector.pData[VECT_Z];
	// Remove angular acceleration from acceleration result
	// Angular acceleration = cross product of velocity and rotation rate

	// Get plane reference gravity vector
	tempVector.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rzx];
	tempVector.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Rzy];
	tempVector.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzz];
	// Calculate roll pitch error
	ahrs_vect_cross_product(&tempVector, &(ahrs_data.GravityVector), &(ahrs_data.RollPitchCorrection));

	// Calculate mag in earth plane
	ahrs_mult_vector_matrix(&(ahrs_data.rotationMatrix), &(ahrs_data.MagVector), &tempVector);
	// Calculate heading
	ahrs_data.Heading = atan2f(tempVector.vector.pData[VECT_X], tempVector.vector.pData[VECT_Y]);
	// Store heading
	MAG_HEADING = (uint16_t)((int16_t)(ahrs_data.Heading * 1000));

	// Store mag vector in earth frame to gravityVector variable
	gravityVector.vector.pData[VECT_X] = tempVector.vector.pData[VECT_X];
	gravityVector.vector.pData[VECT_Y] = tempVector.vector.pData[VECT_Y];
	gravityVector.vector.pData[VECT_Z] = tempVector.vector.pData[VECT_Z];

	// Z = 0
	tempVector.vector.pData[VECT_Z] = 0;
	// Cross with (1,0,0)
	tempVector1.vector.pData[VECT_X] = 1;
	tempVector1.vector.pData[VECT_Y] = 0;
	tempVector1.vector.pData[VECT_Z] = 0;
	ahrs_vect_cross_product(&tempVector1, &tempVector, &tempVector2);
	// Transform to plane coordinates
	// Create matrix transpose
	//ahrs_matrix_transponse(&(ahrs_data.rotationMatrix), &tempMatrix);
	// Move vector to plane coordinate system
	ahrs_mult_vector_matrix_transpose(&(ahrs_data.rotationMatrix), &tempVector2, &(ahrs_data.YawCorrection));

	ahrs_data.Wrp = ahrs_get_vector_norm(&(ahrs_data.RollPitchCorrection)) * 10;
	if(ahrs_data.Wrp < 0)
	{
		ahrs_data.Wrp = -ahrs_data.Wrp;
	}

	if(ahrs_data.Wrp > 1)
	{
		ahrs_data.Wrp = 1;
	}

	ahrs_data.Wy = ahrs_get_vector_norm(&(ahrs_data.YawCorrection)) * 0.5;
	if(ahrs_data.Wy < 0)
	{
		ahrs_data.Wy = -ahrs_data.Wy;
	}
	if(ahrs_data.Wy > 0.5)
	{
		ahrs_data.Wy = 0.5;
	}


	// Add correction to rollPitchCorrection vector
	ahrs_data.totalCorrectionError.vector.pData[VECT_X] = -ahrs_data.RollPitchCorrection.vector.pData[VECT_X];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_X] * ahrs_data.Wy;
	ahrs_data.totalCorrectionError.vector.pData[VECT_Y] = -ahrs_data.RollPitchCorrection.vector.pData[VECT_Y];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Y] * ahrs_data.Wy;
	ahrs_data.totalCorrectionError.vector.pData[VECT_Z] = -ahrs_data.RollPitchCorrection.vector.pData[VECT_Z];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Z] * ahrs_data.Wy;

	// Update PI error regulator
	ahrs_updateVectorPID(&(ahrs_data.PIData), &(ahrs_data.totalCorrectionError), ahrs_data.GyroVector.fDeltaTime);


	// Store update time
	ahrs_data.rotationMatrix.fDataTime = getFTime();

	return SUCCESS;
}

void ahrs_resetPID(PI3Data* PID)
{
	PID->Ix = 0;
	PID->Iy = 0;
	PID->Iz = 0;
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

void ahrs_resetQuaternion(void)
{
	ahrs_data.Q.w = 1;
	ahrs_data.Q.x = 0;
	ahrs_data.Q.y = 0;
	ahrs_data.Q.z = 0;
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
