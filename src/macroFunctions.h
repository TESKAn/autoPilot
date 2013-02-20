/*
 * macroFunctions.h
 *
 *  Created on: Jan 30, 2013
 *      Author: Jure
 */

#ifndef MACROFUNCTIONS_H_
#define MACROFUNCTIONS_H_

#define REPORT_AHRS_Q
//#define REPORT_AHRS_PID

#ifdef REPORT_AHRS_Q
#define REPORT_AHRS	\
	Buffer[0] = 2;\
	Buffer[1] = 5;\
	floatToUint32.f = ahrs_data.Q.w;\
	Buffer[2] = floatToUint32.ch[0];\
	Buffer[3] = floatToUint32.ch[1];\
	Buffer[4] = floatToUint32.ch[2];\
	Buffer[5] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.Q.x;\
	Buffer[6] = floatToUint32.ch[0];\
	Buffer[7] = floatToUint32.ch[1];\
	Buffer[8] = floatToUint32.ch[2];\
	Buffer[9] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.Q.y;\
	Buffer[10] = floatToUint32.ch[0];\
	Buffer[11] = floatToUint32.ch[1];\
	Buffer[12] = floatToUint32.ch[2];\
	Buffer[13] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.Q.z;\
	Buffer[14] = floatToUint32.ch[0];\
	Buffer[15] = floatToUint32.ch[1];\
	Buffer[16] = floatToUint32.ch[2];\
	Buffer[17] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Rx;\
	Buffer[18] = floatToUint32.ch[0];\
	Buffer[19] = floatToUint32.ch[1];\
	Buffer[20] = floatToUint32.ch[2];\
	Buffer[21] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Ry;\
	Buffer[22] = floatToUint32.ch[0];\
	Buffer[23] = floatToUint32.ch[1];\
	Buffer[24] = floatToUint32.ch[2];\
	Buffer[25] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Rz;\
	Buffer[26] = floatToUint32.ch[0];\
	Buffer[27] = floatToUint32.ch[1];\
	Buffer[28] = floatToUint32.ch[2];\
	Buffer[29] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzx];\
	Buffer[30] = floatToUint32.ch[0];\
	Buffer[31] = floatToUint32.ch[1];\
	Buffer[31] = floatToUint32.ch[2];\
	Buffer[33] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzy];\
	Buffer[34] = floatToUint32.ch[0];\
	Buffer[35] = floatToUint32.ch[1];\
	Buffer[36] = floatToUint32.ch[2];\
	Buffer[37] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.rotationMatrix.vector.pData[Rzz];\
	Buffer[38] = floatToUint32.ch[0];\
	Buffer[39] = floatToUint32.ch[1];\
	Buffer[40] = floatToUint32.ch[2];\
	Buffer[41] = floatToUint32.ch[3];\
	floatToUint32.f = gravityVector.vector.pData[VECT_X];\
	Buffer[42] = floatToUint32.ch[0];\
	Buffer[43] = floatToUint32.ch[1];\
	Buffer[44] = floatToUint32.ch[2];\
	Buffer[45] = floatToUint32.ch[3];\
	floatToUint32.f = gravityVector.vector.pData[VECT_Y];\
	Buffer[46] = floatToUint32.ch[0];\
	Buffer[47] = floatToUint32.ch[1];\
	Buffer[48] = floatToUint32.ch[2];\
	Buffer[49] = floatToUint32.ch[3];\
	floatToUint32.f = gravityVector.vector.pData[VECT_Z];\
	Buffer[50] = floatToUint32.ch[0];\
	Buffer[51] = floatToUint32.ch[1];\
	Buffer[52] = floatToUint32.ch[2];\
	Buffer[53] = floatToUint32.ch[3];\
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64)
#endif

#ifdef REPORT_AHRS_PID
#define REPORT_AHRS	\
	Buffer[0] = 2;\
	Buffer[1] = 5;\
	floatToUint32.f = ahrs_data.PIData.Ix;\
	Buffer[2] = floatToUint32.ch[0];\
	Buffer[3] = floatToUint32.ch[1];\
	Buffer[4] = floatToUint32.ch[2];\
	Buffer[5] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Iy;\
	Buffer[6] = floatToUint32.ch[0];\
	Buffer[7] = floatToUint32.ch[1];\
	Buffer[8] = floatToUint32.ch[2];\
	Buffer[9] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Iz;\
	Buffer[10] = floatToUint32.ch[0];\
	Buffer[11] = floatToUint32.ch[1];\
	Buffer[12] = floatToUint32.ch[2];\
	Buffer[13] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Rx;\
	Buffer[18] = floatToUint32.ch[0];\
	Buffer[19] = floatToUint32.ch[1];\
	Buffer[20] = floatToUint32.ch[2];\
	Buffer[21] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Ry;\
	Buffer[22] = floatToUint32.ch[0];\
	Buffer[23] = floatToUint32.ch[1];\
	Buffer[24] = floatToUint32.ch[2];\
	Buffer[25] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.PIData.Rz;\
	Buffer[26] = floatToUint32.ch[0];\
	Buffer[27] = floatToUint32.ch[1];\
	Buffer[28] = floatToUint32.ch[2];\
	Buffer[29] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.magCorrectionError.vector.pData[VECT_X];\
	Buffer[30] = floatToUint32.ch[0];\
	Buffer[31] = floatToUint32.ch[1];\
	Buffer[31] = floatToUint32.ch[2];\
	Buffer[33] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.magCorrectionError.vector.pData[VECT_Y];\
	Buffer[34] = floatToUint32.ch[0];\
	Buffer[35] = floatToUint32.ch[1];\
	Buffer[36] = floatToUint32.ch[2];\
	Buffer[37] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.magCorrectionError.vector.pData[VECT_Z];\
	Buffer[38] = floatToUint32.ch[0];\
	Buffer[39] = floatToUint32.ch[1];\
	Buffer[40] = floatToUint32.ch[2];\
	Buffer[41] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_X];\
	Buffer[42] = floatToUint32.ch[0];\
	Buffer[43] = floatToUint32.ch[1];\
	Buffer[44] = floatToUint32.ch[2];\
	Buffer[45] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_Y];\
	Buffer[46] = floatToUint32.ch[0];\
	Buffer[47] = floatToUint32.ch[1];\
	Buffer[48] = floatToUint32.ch[2];\
	Buffer[49] = floatToUint32.ch[3];\
	floatToUint32.f = ahrs_data.RollPitchCorrection.vector.pData[VECT_Z];\
	Buffer[50] = floatToUint32.ch[0];\
	Buffer[51] = floatToUint32.ch[1];\
	Buffer[52] = floatToUint32.ch[2];\
	Buffer[53] = floatToUint32.ch[3];\
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 64)
#endif
#endif /* MACROFUNCTIONS_H_ */
