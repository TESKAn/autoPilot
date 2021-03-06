/*
 * macroFunctions.h
 *
 *  Created on: Jan 30, 2013
 *      Author: Jure
 */

#ifndef MACROFUNCTIONS_H_
#define MACROFUNCTIONS_H_


// Fast report 0 - DCM matrix and mag vector
#define REPORT_AHRS_DCM_MATRIX_MAG_VECTOR	\
	Buffer[0] = 2;\
	Buffer[1] = 5;\
	Buffer[2] = 1;\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxx];\
	Buffer[3] = floatToUint32.ch[0];\
	Buffer[4] = floatToUint32.ch[1];\
	Buffer[5] = floatToUint32.ch[2];\
	Buffer[6] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryx];\
	Buffer[7] = floatToUint32.ch[0];\
	Buffer[8] = floatToUint32.ch[1];\
	Buffer[9] = floatToUint32.ch[2];\
	Buffer[10] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzx];\
	Buffer[11] = floatToUint32.ch[0];\
	Buffer[12] = floatToUint32.ch[1];\
	Buffer[13] = floatToUint32.ch[2];\
	Buffer[14] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxy];\
	Buffer[15] = floatToUint32.ch[0];\
	Buffer[16] = floatToUint32.ch[1];\
	Buffer[17] = floatToUint32.ch[2];\
	Buffer[18] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryy];\
	Buffer[19] = floatToUint32.ch[0];\
	Buffer[20] = floatToUint32.ch[1];\
	Buffer[21] = floatToUint32.ch[2];\
	Buffer[22] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzy];\
	Buffer[23] = floatToUint32.ch[0];\
	Buffer[24] = floatToUint32.ch[1];\
	Buffer[25] = floatToUint32.ch[2];\
	Buffer[26] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rxz];\
	Buffer[27] = floatToUint32.ch[0];\
	Buffer[28] = floatToUint32.ch[1];\
	Buffer[29] = floatToUint32.ch[2];\
	Buffer[30] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Ryz];\
	Buffer[31] = floatToUint32.ch[0];\
	Buffer[32] = floatToUint32.ch[1];\
	Buffer[33] = floatToUint32.ch[2];\
	Buffer[34] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzz];\
	Buffer[35] = floatToUint32.ch[0];\
	Buffer[36] = floatToUint32.ch[1];\
	Buffer[37] = floatToUint32.ch[2];\
	Buffer[38] = floatToUint32.ch[3];\
	floatToUint32.f = 0;\
	Buffer[39] = floatToUint32.ch[0];\
	Buffer[40] = floatToUint32.ch[1];\
	Buffer[41] = floatToUint32.ch[2];\
	Buffer[42] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_X];\
	Buffer[43] = floatToUint32.ch[0];\
	Buffer[44] = floatToUint32.ch[1];\
	Buffer[45] = floatToUint32.ch[2];\
	Buffer[46] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Y];\
	Buffer[47] = floatToUint32.ch[0];\
	Buffer[48] = floatToUint32.ch[1];\
	Buffer[49] = floatToUint32.ch[2];\
	Buffer[50] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Z];\
	Buffer[51] = floatToUint32.ch[0];\
	Buffer[52] = floatToUint32.ch[1];\
	Buffer[53] = floatToUint32.ch[2];\
	Buffer[54] = floatToUint32.ch[3];\
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64)

// Fast report 1 - accelerometer, gyroscope, magnetometer
#define REPORT_AHRS_ACC_GYRO_MAG_TOTALCORRECTION	\
	Buffer[0] = 2;\
	Buffer[1] = 5;\
	Buffer[2] = 2;\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_X];\
	Buffer[3] = floatToUint32.ch[0];\
	Buffer[4] = floatToUint32.ch[1];\
	Buffer[5] = floatToUint32.ch[2];\
	Buffer[6] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Y];\
	Buffer[7] = floatToUint32.ch[0];\
	Buffer[8] = floatToUint32.ch[1];\
	Buffer[9] = floatToUint32.ch[2];\
	Buffer[10] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Z];\
	Buffer[11] = floatToUint32.ch[0];\
	Buffer[12] = floatToUint32.ch[1];\
	Buffer[13] = floatToUint32.ch[2];\
	Buffer[14] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_X];\
	Buffer[15] = floatToUint32.ch[0];\
	Buffer[16] = floatToUint32.ch[1];\
	Buffer[17] = floatToUint32.ch[2];\
	Buffer[18] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Y];\
	Buffer[19] = floatToUint32.ch[0];\
	Buffer[20] = floatToUint32.ch[1];\
	Buffer[21] = floatToUint32.ch[2];\
	Buffer[22] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Z];\
	Buffer[23] = floatToUint32.ch[0];\
	Buffer[24] = floatToUint32.ch[1];\
	Buffer[25] = floatToUint32.ch[2];\
	Buffer[26] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_X];\
	Buffer[27] = floatToUint32.ch[0];\
	Buffer[28] = floatToUint32.ch[1];\
	Buffer[29] = floatToUint32.ch[2];\
	Buffer[30] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Y];\
	Buffer[31] = floatToUint32.ch[0];\
	Buffer[32] = floatToUint32.ch[1];\
	Buffer[33] = floatToUint32.ch[2];\
	Buffer[34] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Z];\
	Buffer[35] = floatToUint32.ch[0];\
	Buffer[36] = floatToUint32.ch[1];\
	Buffer[37] = floatToUint32.ch[2];\
	Buffer[38] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_X];\
	Buffer[39] = floatToUint32.ch[0];\
	Buffer[40] = floatToUint32.ch[1];\
	Buffer[41] = floatToUint32.ch[2];\
	Buffer[42] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Y];\
	Buffer[43] = floatToUint32.ch[0];\
	Buffer[44] = floatToUint32.ch[1];\
	Buffer[45] = floatToUint32.ch[2];\
	Buffer[46] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Z];\
	Buffer[47] = floatToUint32.ch[0];\
	Buffer[48] = floatToUint32.ch[1];\
	Buffer[49] = floatToUint32.ch[2];\
	Buffer[50] = floatToUint32.ch[3];\
	floatToUint32.f = 0;\
	Buffer[51] = floatToUint32.ch[0];\
	Buffer[52] = floatToUint32.ch[1];\
	Buffer[53] = floatToUint32.ch[2];\
	Buffer[54] = floatToUint32.ch[3];\
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64)

// Fast report 1 - accelerometer, gyroscope, magnetometer
#define REPORT_AHRS_ACC_GYRO_MAG_TOTALCORRECTION	\
	Buffer[0] = 2;\
	Buffer[1] = 5;\
	Buffer[2] = 2;\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_X];\
	Buffer[3] = floatToUint32.ch[0];\
	Buffer[4] = floatToUint32.ch[1];\
	Buffer[5] = floatToUint32.ch[2];\
	Buffer[6] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Y];\
	Buffer[7] = floatToUint32.ch[0];\
	Buffer[8] = floatToUint32.ch[1];\
	Buffer[9] = floatToUint32.ch[2];\
	Buffer[10] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.AccVector.vector.pData[VECT_Z];\
	Buffer[11] = floatToUint32.ch[0];\
	Buffer[12] = floatToUint32.ch[1];\
	Buffer[13] = floatToUint32.ch[2];\
	Buffer[14] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_X];\
	Buffer[15] = floatToUint32.ch[0];\
	Buffer[16] = floatToUint32.ch[1];\
	Buffer[17] = floatToUint32.ch[2];\
	Buffer[18] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Y];\
	Buffer[19] = floatToUint32.ch[0];\
	Buffer[20] = floatToUint32.ch[1];\
	Buffer[21] = floatToUint32.ch[2];\
	Buffer[22] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.GyroVector.vector.pData[VECT_Z];\
	Buffer[23] = floatToUint32.ch[0];\
	Buffer[24] = floatToUint32.ch[1];\
	Buffer[25] = floatToUint32.ch[2];\
	Buffer[26] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_X];\
	Buffer[27] = floatToUint32.ch[0];\
	Buffer[28] = floatToUint32.ch[1];\
	Buffer[29] = floatToUint32.ch[2];\
	Buffer[30] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Y];\
	Buffer[31] = floatToUint32.ch[0];\
	Buffer[32] = floatToUint32.ch[1];\
	Buffer[33] = floatToUint32.ch[2];\
	Buffer[34] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.MagVector.vector.pData[VECT_Z];\
	Buffer[35] = floatToUint32.ch[0];\
	Buffer[36] = floatToUint32.ch[1];\
	Buffer[37] = floatToUint32.ch[2];\
	Buffer[38] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_X];\
	Buffer[39] = floatToUint32.ch[0];\
	Buffer[40] = floatToUint32.ch[1];\
	Buffer[41] = floatToUint32.ch[2];\
	Buffer[42] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Y];\
	Buffer[43] = floatToUint32.ch[0];\
	Buffer[44] = floatToUint32.ch[1];\
	Buffer[45] = floatToUint32.ch[2];\
	Buffer[46] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.totalCorrectionError.vector.pData[VECT_Z];\
	Buffer[47] = floatToUint32.ch[0];\
	Buffer[48] = floatToUint32.ch[1];\
	Buffer[49] = floatToUint32.ch[2];\
	Buffer[50] = floatToUint32.ch[3];\
	floatToUint32.f = 0;\
	Buffer[51] = floatToUint32.ch[0];\
	Buffer[52] = floatToUint32.ch[1];\
	Buffer[53] = floatToUint32.ch[2];\
	Buffer[54] = floatToUint32.ch[3];\
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64)

#endif /* MACROFUNCTIONS_H_ */
