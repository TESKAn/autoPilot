/*
 * ahrs.c
 *
 *  Created on: Dec 28, 2012
 *      Author: Jure
 */


#include "allinclude.h"
#include "var.h"

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
	ahrsStructure->Q.dataTime = getSystemTime();
	ahrsStructure->Q.deltaTime = 0;
	math_vector3fDataInit(&(ahrsStructure->AccVector), ROW);
	math_vector3qDataInit(&(ahrsStructure->AccOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->AccScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GravityVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GyroVector), ROW);
	math_vector3qDataInit(&(ahrsStructure->GyroOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GyroScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->GyroValueAdjusted), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagOffsetVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagScaleVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->MagPreviousResult), ROW);
	ahrsStructure->MagOffsetCalcGain = 0.1f;
	math_vector3fDataInit(&(ahrsStructure->MagInEarthFrame), ROW);
	math_vector3fDataInit(&(ahrsStructure->RollPitchYaw), ROW);
	math_matrix3by3Init(&(ahrsStructure->rotationMatrix), MATH_YES);
	math_matrix3by3Init(&(ahrsStructure->magRotationMatrix), MATH_YES);
	ahrsStructure->GPSData.altitude = 0;
	ahrsStructure->GPSData.dataTime = getSystemTime();
	ahrsStructure->GPSData.dataStartTime = getSystemTime();
	ahrsStructure->GPSData.dataValid = INVALID;
	ahrsStructure->GPSData.latitude = 0;
	ahrsStructure->GPSData.longitude = 0;
	ahrsStructure->GPSData.speed = 0;
	ahrsStructure->GPSData.trackAngle = 0;
	math_matrix3by3Init(&(ahrsStructure->GPSReference), MATH_NO);
	ahrsStructure->Altitude.currentAltitude = 0;
	ahrsStructure->Altitude.dataTime = getSystemTime();
	ahrsStructure->Altitude.deltaTime = 0;
	ahrsStructure->Altitude.lastAltitude = 0;
	ahrsStructure->Altitude.verticalAcceleration = 0;
	ahrsStructure->Altitude.verticalSpeed = 0;
	ahrs_resetRotationMatrix();
	ahrsStructure->PlaneSpeed = 0;
	math_vector3fDataInit(&(ahrsStructure->PlaneSpeedVector), ROW);
	math_vector3fDataInit(&(ahrsStructure->AccelerometerIntegral), ROW);
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
	ahrsStructure->PIData.dataTime = getSystemTime();
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

	/*
	ahrsStructure->MagOffsetVector.vector.pData[VECT_X] = 0;
	ahrsStructure->MagOffsetVector.vector.pData[VECT_Y] = 0;
	ahrsStructure->MagOffsetVector.vector.pData[VECT_Z] = 0;
	*/
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
	ahrsStructure->sampleDiscardCount = DEFAULT_DISCARD_COUNT;
	ahrsStructure->PIDErrorThreshold = DEFAULT_PID_ERROR_THRESHOLD;

}

void ahrs_updateGyroReading(void)
{
	updateScaledVector(&(ahrs_data.GyroVector), GYRO_X, GYRO_Y, GYRO_Z, ahrs_data.gyroRate * DEG_TO_RAD);
}

void ahrs_updateAccReading(void)
{
	updateScaledVector(&(ahrs_data.AccVector), ACC_X, ACC_Y, ACC_Z, ahrs_data.accRate);
	// Invert to get gravity - acceleration
	ahrs_data.AccVector.vector.pData[VECT_X] = -ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.AccVector.vector.pData[VECT_Y] = -ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.AccVector.vector.pData[VECT_Z] = -ahrs_data.AccVector.vector.pData[VECT_Z];
}

void ahrs_updateMagReading(void)
{
	/*
	float32_t magDifference = 0;
	// Store previous vector
	ahrs_copy_vector(&(ahrs_data.MagVector), &(ahrs_data.MagPreviousResult));
	// Calculate fixed vector = current vector - offset
	// Calculate new vector
	updateScaledVector(&(ahrs_data.MagVector), MAG_X, MAG_Y, MAG_Z, ahrs_data.magRate);
	// Normalize
	//ahrs_normalize_vector(&(ahrs_data.MagVector));
	// Remove offset
	ahrs_vector_substract(&(ahrs_data.MagVector), &(ahrs_data.MagOffsetVector), &(ahrs_data.MagVector));
	// Update offset
	// Calculate difference
	ahrs_vector_substract(&(ahrs_data.MagVector), &(ahrs_data.MagPreviousResult), &tempVector);
	// Normalize difference
	ahrs_normalize_vector(&tempVector);
	// Calculate difference of magnitudes
	magDifference = ahrs_vector_magnitude(&(ahrs_data.MagVector)) - ahrs_vector_magnitude(&(ahrs_data.MagPreviousResult));
	// Check difference magnitude, if below limit, do nothing
	if(magDifference > 0.0005f)
	{
		// Multiply by gain
		magDifference = magDifference * ahrs_data.MagOffsetCalcGain;
		// Calculate offset correction
		ahrs_mult_vector_scalar(&tempVector, magDifference);
		// Add to offset
		ahrs_vector_add(&tempVector, &(ahrs_data.MagOffsetVector), &(ahrs_data.MagOffsetVector));
	}
*/
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

	// Turn to earth frame
	ahrs_mult_vector_matrix(&(ahrs_data.rotationMatrix), &(ahrs_data.MagVector), &(ahrs_data.MagInEarthFrame));
}

// 10.4.2013
// Function to update plane speed in earth coordinates
// And plane X axis speed
void ahrs_updateAccVelocityReading(void)
{
	// Calculate plane X speed change
	// dV = a * dT
	float32_t dV = 0;
	float32_t dT = 0;
	float32_t temp = 0;
	// Multiply because it is faster
	dT = (float32_t)ahrs_data.AccVector.deltaTime * 0.01f;
	// Calculate acceleration in plane X axis
	temp = ahrs_data.AccVector.vector.pData[VECT_X] - ahrs_data.rotationMatrix.vector.pData[Rxz];
	// Calculate dV in plane X direction
	dV = temp * dT;
	// Add to plane V
	ahrs_data.PlaneSpeed = ahrs_data.PlaneSpeed + dV;
	// Transform acceleration vector to earth frame
	ahrs_mult_vector_matrix(&(ahrs_data.rotationMatrix), &(ahrs_data.AccVector), &tempVector);
	// Update acceleration integral
	dV = tempVector.vector.pData[VECT_X] * dT;
	ahrs_data.AccelerometerIntegral.vector.pData[VECT_X] = ahrs_data.AccelerometerIntegral.vector.pData[VECT_X] + dV;
	dV = tempVector.vector.pData[VECT_Y] * dT;
	ahrs_data.AccelerometerIntegral.vector.pData[VECT_Y] = ahrs_data.AccelerometerIntegral.vector.pData[VECT_Y] + dV;
	dV = tempVector.vector.pData[VECT_Z] * dT;
	ahrs_data.AccelerometerIntegral.vector.pData[VECT_Z] = ahrs_data.AccelerometerIntegral.vector.pData[VECT_Z] + dV;
	// Remove gravity
	tempVector.vector.pData[VECT_Z] = tempVector.vector.pData[VECT_Z] + 1;
	// Calculate and add dV's for each earth axis
	ahrs_data.PlaneSpeedVector.vector.pData[VECT_X] = ahrs_data.AccelerometerIntegral.vector.pData[VECT_X];
	ahrs_data.PlaneSpeedVector.vector.pData[VECT_Y] = ahrs_data.AccelerometerIntegral.vector.pData[VECT_Y];
	dV = tempVector.vector.pData[VECT_Z] * dT;
	ahrs_data.PlaneSpeedVector.vector.pData[VECT_Z] = ahrs_data.PlaneSpeedVector.vector.pData[VECT_Z] + dV;
}

void ahrs_update_altitude(void)
{
	uint32_t timeCalculation = getSystemTime();
	float deltaTime = timeCalculation / 100;
	ahrs_data.Altitude.currentAltitude = (float)((int16_t)BARO) + (float)BARO_FRAC/10;
	ahrs_data.Altitude.deltaTime = timeCalculation - ahrs_data.Altitude.dataTime;
	ahrs_data.Altitude.dataTime = timeCalculation;
	ahrs_data.Altitude.verticalSpeed = (ahrs_data.Altitude.lastAltitude - ahrs_data.Altitude.currentAltitude) / (deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.verticalAcceleration = ahrs_data.Altitude.verticalSpeed / (deltaTime * SYSTIME_TOSECONDS);
	ahrs_data.Altitude.lastAltitude = ahrs_data.Altitude.currentAltitude;
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
	// Acceleration
	ahrs_updateAccReading();
	// Update plane speed
	ahrs_updateAccVelocityReading();
	// Update gyros
	ahrs_updateGyroReading();
	// Update altitude reading
	ahrs_update_altitude();
	// Update mag reading
	ahrs_updateMagReading();

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
	dT = (float)(data->GyroVector.deltaTime) * SYSTIME_TOSECONDS;
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
		if(status == ARM_MATH_SUCCESS)
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
			return ERROR;
		}
	}
	else
	{
		return SUCCESS;
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
    /*
    Qtemp.x = xx + (Qtemp.w * dWx + yy * dWz - zz * dWy)*(0.5 * dT);
    Qtemp.y = yy + (Qtemp.w * dWy - xx * dWz + zz * dWx)*(0.5 * dT);
    Qtemp.z = zz + (Qtemp.w * dWz + xx * dWy - yy * dWx)*(0.5 * dT);
    */
    Qtemp.x = xx + (Qtemp.w * dWx + yy * dWz - zz * dWy)*(0.5 * dT);
    Qtemp.y = yy + (Qtemp.w * dWy - xx * dWz + zz * dWx)*(0.5 * dT);
    Qtemp.z = zz + (Qtemp.w * dWz + xx * dWy - yy * dWx)*(0.5 * dT);

    ahrs_data.Q.w = Qtemp.w;
    ahrs_data.Q.x = Qtemp.x;
    ahrs_data.Q.y = Qtemp.y;
    ahrs_data.Q.z = Qtemp.z;

    // Normalize quaternion
    // Calculate squares
    wx = ahrs_data.Q.w * ahrs_data.Q.w;
    xx = ahrs_data.Q.x * ahrs_data.Q.x;
    yy = ahrs_data.Q.y * ahrs_data.Q.y;
    zz = ahrs_data.Q.z * ahrs_data.Q.z;
    // Calculate root
    wy = sqrtf(wx + xx + yy + zz);
    // Calculate multiplication factor
    wz = 1 / wy;
    // Normalize
    ahrs_data.Q.w = ahrs_data.Q.w * wz;
    ahrs_data.Q.x = ahrs_data.Q.x * wz;
    ahrs_data.Q.y = ahrs_data.Q.y * wz;
    ahrs_data.Q.z = ahrs_data.Q.z * wz;


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
	ahrs_data.rotationMatrix.vector.pData[Rxz] = xz - wy;
	ahrs_data.rotationMatrix.vector.pData[Ryz] = yz + wx;
	ahrs_data.rotationMatrix.vector.pData[Rzz] = 1 - xx - yy;


	// New way of updating data with reference vectors
	// Get gravity vector = Z ref
	ahrs_data.GravityVector.vector.pData[VECT_X] = ahrs_data.AccVector.vector.pData[VECT_X];
	ahrs_data.GravityVector.vector.pData[VECT_Y] = ahrs_data.AccVector.vector.pData[VECT_Y];
	ahrs_data.GravityVector.vector.pData[VECT_Z] = ahrs_data.AccVector.vector.pData[VECT_Z];
	// Normalize
	ahrs_normalize_vector(&(ahrs_data.GravityVector));
	// Calculate cross product of gravity X mag vector to get earth Y axis
	ahrs_vect_cross_product(&(ahrs_data.GravityVector), &(ahrs_data.MagVector), &tempVector);
	// Normalize
	ahrs_normalize_vector(&tempVector);
	// Calculate X axis from gravity and y
	ahrs_vect_cross_product(&tempVector, &(ahrs_data.GravityVector), &tempVector1);
	// Normalize
	ahrs_normalize_vector(&tempVector1);

	// Calculate Z axis error
	// Store earth Z estimated, as seen in earth coordinates
	tempVector2.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rxz];
	tempVector2.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Ryz];
	tempVector2.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzz];
	// Calculate error
	ahrs_vect_cross_product(&tempVector2, &(ahrs_data.GravityVector), &(ahrs_data.RollPitchCorrection));

	// Calculate X axis error
	// Store earth X estimate
	tempVector2.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rxx];
	tempVector2.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Ryx];
	tempVector2.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzx];
	// Calculate error
	ahrs_vect_cross_product(&tempVector2, &tempVector1, &(ahrs_data.YawCorrection));


/*

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
	ahrs_mult_vector_matrix_transpose(&(ahrs_data.rotationMatrix), &(ahrs_data.MagVector), &tempVector);

	// Z = 0
	tempVector.vector.pData[VECT_Z] = 0;

	// Normalize vector
	ahrs_normalize_vector(&tempVector);
	// Calculate heading
	ahrs_data.Heading = atan2f(tempVector.vector.pData[VECT_X], tempVector.vector.pData[VECT_Y]);
	// Convert to 0 - 360
	if(ahrs_data.Heading < 0)
	{
		ahrs_data.Heading = ahrs_data.Heading + 2*PI;
	}
	// Store heading
	MAG_HEADING = (uint16_t)((int16_t)(ahrs_data.Heading * 1000));

	// Store mag vector in earth frame to gravityVector variable
	gravityVector.vector.pData[VECT_X] = tempVector.vector.pData[VECT_X];
	gravityVector.vector.pData[VECT_Y] = tempVector.vector.pData[VECT_Y];
	gravityVector.vector.pData[VECT_Z] = tempVector.vector.pData[VECT_Z];

	xx = cosf(ahrs_data.Heading);	// COG X
	yy = sinf(ahrs_data.Heading);	// COG Y

	xy = ahrs_data.rotationMatrix.vector.pData[Rxx] * yy;
	xz = ahrs_data.rotationMatrix.vector.pData[Ryx] * xx;

	xx = xy - xz;
	ahrs_data.YawCorrection.vector.pData[VECT_X] = ahrs_data.rotationMatrix.vector.pData[Rzx];
	ahrs_data.YawCorrection.vector.pData[VECT_Y] = ahrs_data.rotationMatrix.vector.pData[Rzy];
	ahrs_data.YawCorrection.vector.pData[VECT_Z] = ahrs_data.rotationMatrix.vector.pData[Rzz];
	ahrs_mult_vector_scalar(&(ahrs_data.YawCorrection), xx);


	// Cross with (1,0,0)
	tempVector1.vector.pData[VECT_X] = 1;
	tempVector1.vector.pData[VECT_Y] = 0;
	tempVector1.vector.pData[VECT_Z] = 0;
	ahrs_vect_cross_product(&tempVector, &tempVector1, &tempVector2);
	// Transform to plane coordinates
	// Create matrix transpose
	//ahrs_matrix_transponse(&(ahrs_data.rotationMatrix), &tempMatrix);
	// Move vector to plane coordinate system
	ahrs_mult_vector_matrix_transpose(&(ahrs_data.rotationMatrix), &tempVector2, &(ahrs_data.YawCorrection));
*/
	ahrs_data.Wrp = ahrs_get_vector_norm(&(ahrs_data.RollPitchCorrection)) * 0.1;
	if(ahrs_data.Wrp < 0)
	{
		ahrs_data.Wrp = -ahrs_data.Wrp;
	}

	if(ahrs_data.Wrp > 0.1)
	{
		ahrs_data.Wrp = 0.1;
	}

	ahrs_data.Wy = ahrs_get_vector_norm(&(ahrs_data.YawCorrection)) * 0.1;
	if(ahrs_data.Wy < 0)
	{
		ahrs_data.Wy = -ahrs_data.Wy;
	}
	if(ahrs_data.Wy > 0.1)
	{
		ahrs_data.Wy = 0.1;
	}


	// Add correction to rollPitchCorrection vector
	ahrs_data.totalCorrectionError.vector.pData[VECT_X] = ahrs_data.RollPitchCorrection.vector.pData[VECT_X];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_X] * ahrs_data.Wy;
	ahrs_data.totalCorrectionError.vector.pData[VECT_Y] = ahrs_data.RollPitchCorrection.vector.pData[VECT_Y];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Y] * ahrs_data.Wy;
	ahrs_data.totalCorrectionError.vector.pData[VECT_Z] = ahrs_data.RollPitchCorrection.vector.pData[VECT_Z];// * ahrs_data.Wrp + ahrs_data.YawCorrection.vector.pData[VECT_Z] * ahrs_data.Wy;

	// Update PI error regulator
	ahrs_updateVectorPID(&(ahrs_data.PIData), &(ahrs_data.totalCorrectionError), ahrs_data.GyroVector.deltaTime);


	// Store update time
	ahrs_data.rotationMatrix.dataTime = getSystemTime();

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
	// Only add if error is larger than 0.0001
	//if(error > 0.0001f)
	{
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
		//if(error > 0.0001f)
		{
			// Multiply with X
			rotMatrix->vector.pData[Rxx] = rotMatrix->vector.pData[Rxx] * error;
			rotMatrix->vector.pData[Rxy] = rotMatrix->vector.pData[Rxy] * error;
			rotMatrix->vector.pData[Rxz] = rotMatrix->vector.pData[Rxz] * error;
		}
		// Calculate scalar Y.Y
		error = (rotMatrix->vector.pData[Ryx] * rotMatrix->vector.pData[Ryx])+(rotMatrix->vector.pData[Ryy] * rotMatrix->vector.pData[Ryy])+(rotMatrix->vector.pData[Ryz] * rotMatrix->vector.pData[Ryz]);
		// Calculate 3 - scalar
		error = 3 - error;
		// Calculate one half
		error = error / 2;
		//if(error > 0.0001f)
		{
			// Multiply with Y
			rotMatrix->vector.pData[Ryx] = rotMatrix->vector.pData[Ryx] * error;
			rotMatrix->vector.pData[Ryy] = rotMatrix->vector.pData[Ryy] * error;
			rotMatrix->vector.pData[Ryz] = rotMatrix->vector.pData[Ryz] * error;
		}
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
		//if(error > 0.0001f)
		{
			// Multiply with Z
			rotMatrix->vector.pData[Rzx] = rotMatrix->vector.pData[Rzx] * error;
			rotMatrix->vector.pData[Rzy] = rotMatrix->vector.pData[Rzy] * error;
			rotMatrix->vector.pData[Rzz] = rotMatrix->vector.pData[Rzz] * error;
		}
	}
}

void ahrs_getAngles(matrix3by3 * rotMatrix, vector3f *vector)
{
	// Store angles
	vector->pData[VECT_X] = -asin(rotMatrix->vector.pData[Rzx]) * 1000;
	vector->pData[VECT_Y] = atan2(rotMatrix->vector.pData[Rzy],  rotMatrix->vector.pData[Rzz]) * 1000;
	vector->pData[VECT_Z] = atan2(rotMatrix->vector.pData[Ryx],  rotMatrix->vector.pData[Rxx]) * 1000;
}
