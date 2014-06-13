/*
 * usb.c
 *
 *  Created on: Dec 13, 2012
 *      Author: XPMUser
 */

#include "allinclude.h"

float64_t* USBFloat64Vars1[7];

float32_t* USBFloat32Vars1[15];

// Array of int16_t pointers
int16_t* USBInt16Vars1[30];

// uint16
uint16_t* USBUint16Vars1[30];

// int32
int32_t* USBInt32Vars1[15];

// uint32
uint32_t* USBUint32Vars1[15];


void usb_initVars(void)
{
	// Store pointers to variables of interest
	USBFloat64Vars1[0] = &fusionData._gps.latitude;
	USBFloat64Vars1[1] = &fusionData._gps.longitude;

	USBFloat32Vars1[0] = &fusionData._gyro.vector.x;
	USBFloat32Vars1[1] = &fusionData._gyro.vector.y;
	USBFloat32Vars1[2] = &fusionData._gyro.vector.z;

	USBFloat32Vars1[3] = &fusionData._accelerometer.vector.x;
	USBFloat32Vars1[4] = &fusionData._accelerometer.vector.y;
	USBFloat32Vars1[5] = &fusionData._accelerometer.vector.z;

	USBFloat32Vars1[6] = &fusionData._mag.vector.x;
	USBFloat32Vars1[7] = &fusionData._mag.vector.y;
	USBFloat32Vars1[8] = &fusionData._mag.vector.z;

	USBFloat32Vars1[9] = &fusionData._altimeter.pressure;
	USBFloat32Vars1[10] = &fusionData._altimeter.altitude;

	USBFloat32Vars1[11] = &fusionData.ROLLPITCHYAW.roll;
	USBFloat32Vars1[12] = &fusionData.ROLLPITCHYAW.pitch;
	USBFloat32Vars1[13] = &fusionData.ROLLPITCHYAW.yaw;

}
