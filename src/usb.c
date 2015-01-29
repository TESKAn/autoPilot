/*
 * usb.c
 *
 *  Created on: Dec 13, 2012
 *      Author: XPMUser
 */

#include "allinclude.h"

float64_t* USBFloat64Vars1[7];

float32_t* USBFloat32Vars1[15];
float32_t* USBFloat32Vars2[15];
float32_t* USBFloat32Vars3[15];

// int32
int32_t* USBInt32Vars1[15];

// uint32
uint32_t* USBUint32Vars1[15];

// Array of int16_t pointers
int16_t* USBInt16Vars1[30];

// uint16
uint16_t* USBUint16Vars1[30];

// int8
int8_t* USBInt8Vars1[60];

// uint8
uint8_t* USBUint8Vars1[60];


void usb_initVars(void)
{

	// Store pointers to variables of interest
	// USBFloat64Vars1
	USBFloat64Vars1[0] = &fusionData._gps.latitude;
	USBFloat64Vars1[1] = &fusionData._gps.longitude;

	// USBFloat32Vars1
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
	USBFloat32Vars1[14] = &fusionData._gyro.fDeltaTime;


	// USBUint32Vars1
	USBUint32Vars1[0] = &fusionData._gps.dataTime;
	USBUint32Vars1[1] = &fusionData._gps.deltaTime;
	USBUint32Vars1[2] = &fusionData.sensorInterruptDeltaTime;
	USBUint32Vars1[3] = &fusionData.sensorInterruptTime;

	// USBUint16Vars1
	USBUint16Vars1[0] = &fusionData._gps.satStatus;
	USBUint16Vars1[1] = &fusionData._gps.miliseconds;
	USBUint16Vars1[2] = &fusionData._gps.valid;
	USBUint16Vars1[3] = &fusionData._gps.ns_ew;
	USBUint16Vars1[4] = &fusionData._gps.magvar_ew;
	USBUint16Vars1[5] = &mainLoopState;
	USBUint16Vars1[6] = &RCData.PWMIN_1;
	USBUint16Vars1[7] = &RCData.PWMIN_2;
	USBUint16Vars1[8] = &RCData.PWMIN_3;
	USBUint16Vars1[9] = &RCData.PWMIN_4;
	USBUint16Vars1[10] = &RCData.PWMIN_5;
	USBUint16Vars1[11] = &RCData.PWMIN_6;
	USBUint16Vars1[12] = &RCData.PWMIN_7;
	USBUint16Vars1[13] = &RCData.PWMIN_8;
	USBUint16Vars1[14] = &RCData.PWMOUT_1;
	USBUint16Vars1[15] = &RCData.PWMOUT_2;
	USBUint16Vars1[16] = &RCData.PWMOUT_3;
	USBUint16Vars1[17] = &RCData.PWMOUT_4;
	USBUint16Vars1[18] = &RCData.PWMOUT_5;
	USBUint16Vars1[19] = &RCData.PWMOUT_6;
	USBUint16Vars1[20] = &RCData.PWMOUT_7;
	USBUint16Vars1[21] = &RCData.PWMOUT_8;
	USBUint16Vars1[22] = &RCData.PWMOUT_9;
	USBUint16Vars1[23] = &RCData.PWMOUT_10;
	USBUint16Vars1[24] = &RCData.PWMOUT_11;
	USBUint16Vars1[25] = &RCData.PWMOUT_12;


	// USBUint8Vars1
	USBUint8Vars1[0] = &fusionData._gps.hours;
	USBUint8Vars1[1] = &fusionData._gps.minutes;
	USBUint8Vars1[2] = &fusionData._gps.seconds;
	USBUint8Vars1[3] = &fusionData._gps.day;
	USBUint8Vars1[4] = &fusionData._gps.month;
	USBUint8Vars1[5] = &fusionData._gps.year;
	USBUint8Vars1[6] = &fusionData._gps.dataOK;

	// USBFloat32Vars2
	USBFloat32Vars2[0] = &fusionData._gps.speed;
	USBFloat32Vars2[1] = &fusionData._gps.altitude;
	USBFloat32Vars2[2] = &fusionData._gps.trackAngle;
	USBFloat32Vars2[3] = &fusionData._gps.hdop;
	USBFloat32Vars2[4] = &fusionData._gps.gg;
	USBFloat32Vars2[5] = &fusionData._gps.magvar;
	USBFloat32Vars2[6] = &fusionData._gps.speed3D.x;
	USBFloat32Vars2[7] = &fusionData._gps.speed3D.y;
	USBFloat32Vars2[8] = &fusionData._gps.speed3D.z;
	USBFloat32Vars2[9] = &fusionData._mag.heading;
	USBFloat32Vars2[10] = &fusionData._mag.vectorRaw.x;
	USBFloat32Vars2[11] = &fusionData._mag.vectorRaw.y;
	USBFloat32Vars2[12] = &fusionData._mag.vectorRaw.z;


	// USBFloat32Vars3
	USBFloat32Vars3[0] = &fusionData._fusion_DCM.a.x;
	USBFloat32Vars3[1] = &fusionData._fusion_DCM.a.y;
	USBFloat32Vars3[2] = &fusionData._fusion_DCM.a.z;

	USBFloat32Vars3[3] = &fusionData._fusion_DCM.b.x;
	USBFloat32Vars3[4] = &fusionData._fusion_DCM.b.y;
	USBFloat32Vars3[5] = &fusionData._fusion_DCM.b.z;

	USBFloat32Vars3[6] = &fusionData._fusion_DCM.c.x;
	USBFloat32Vars3[7] = &fusionData._fusion_DCM.c.y;
	USBFloat32Vars3[8] = &fusionData._fusion_DCM.c.z;

}
