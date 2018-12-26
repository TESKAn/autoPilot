/*
 * can.c
 *
 *  Created on: 6. jan. 2017
 *      Author: jmoc
 */

#include "allinclude.h"

//#include "stm32f4xx_can.h"
//#include "stm32f4xx_rcc.h"
//#include "customTypedefs.h"
#include "can.h"

CANSTRUCT CANData;

//********************************
// Hardware - dependent
void InitCANLink()
{
	// Init CAN filter(s) for rx messages
	CAN_FilterInitTypeDef CANFilterInitStruct;
	CANFilterInitStruct.CAN_FilterIdHigh = 0;//0x004e;
	CANFilterInitStruct.CAN_FilterIdLow = 0;//0x2123;
	CANFilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;
	CANFilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
	CANFilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CANFilterInitStruct.CAN_FilterNumber = 0;
	CANFilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CANFilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CANFilterInitStruct.CAN_FilterActivation = ENABLE;

	CAN_FilterInit(&CANFilterInitStruct);
}

uint32_t CAN_GenerateID(uint32_t ui32PRIO, uint32_t ui32MID)
{
	uint32_t ui32Temp = 0;
	uint32_t ui32MsgID = 0;

	// Generate message ID
	ui32Temp = ui32PRIO;
	ui32Temp = ui32Temp << 24;
	ui32MsgID = ui32Temp;

	ui32Temp = ui32MID;
	ui32Temp = ui32Temp << 8;
	ui32MsgID = ui32MsgID | ui32Temp;

	ui32Temp = CAN_ID;
	ui32MsgID = ui32MsgID | ui32Temp;

	return ui32MsgID;
}

int16_t CAN_SendMinMaxRPM()
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPMLIMIT, CAN_MID_SETRPMLIMIT);

	// Destination ID - 0 to all
	msg.Data[0] = 0x0;
	msg.Data[1] = 0x0;
	// Min RPM
	cnvrt_number.i16[0] = 2000;
	msg.Data[2] = cnvrt_number.ch[0];
	msg.Data[3] = cnvrt_number.ch[1];
	// Max RPM
	cnvrt_number.i16[0] = 20000;
	msg.Data[4] = cnvrt_number.ch[0];
	msg.Data[5] = cnvrt_number.ch[1];

	msg.Data[6] = 0x0;
	msg.Data[7] = 0xc0;
	msg.DLC = 8;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendRPM(int16_t i16Dest, uint16_t ui16RPM1, uint16_t ui16RPM2)
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM, CAN_MID_SETRPM);

	// Destination ID - 0 to all
	cnvrt_number.i16[0] = i16Dest;
	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	// RPM 1
	cnvrt_number.ui16[0] = ui16RPM1;
	msg.Data[2] = cnvrt_number.ch[0];
	msg.Data[3] = cnvrt_number.ch[1];
	// Max RPM
	cnvrt_number.ui16[0] = ui16RPM2;
	msg.Data[4] = cnvrt_number.ch[0];
	msg.Data[5] = cnvrt_number.ch[1];

	msg.Data[6] = 0x0;
	msg.Data[7] = 0xc0;
	msg.DLC = 8;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendOrientation()
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	uint16_t ui16Roll = Float32ToFloat16(FCFlightData.ORIENTATION.f32Roll);
	uint16_t ui16Pitch = Float32ToFloat16(FCFlightData.ORIENTATION.f32Pitch);
	uint16_t ui16Yaw = Float32ToFloat16(FCFlightData.ORIENTATION.f32Yaw);

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_ORIENTATION, CAN_MID_ORIENTATION);

	cnvrt_number.ui16[0] = ui16Roll;
	cnvrt_number.ui16[1] = ui16Pitch;
	cnvrt_number.ui16[2] = ui16Yaw;

	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	msg.Data[2] = cnvrt_number.ch[2];
	msg.Data[3] = cnvrt_number.ch[3];
	msg.Data[4] = cnvrt_number.ch[4];
	msg.Data[5] = cnvrt_number.ch[5];
	msg.Data[6] = 0xc0;
	msg.Data[7] = 0xc0;
	msg.DLC = 7;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendNodeStatus()
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;
	// Setup data in buffer
	cnvrt_number.ui32[0] = systemTime / 1000;

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_STATUS, CAN_MID_STATUS);

	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	msg.Data[2] = cnvrt_number.ch[2];
	msg.Data[3] = cnvrt_number.ch[3];
	msg.Data[4] = 0x00;
	msg.Data[5] = 0x00;
	msg.Data[6] = 0x00;
	msg.Data[7] = 0xc0;
	msg.DLC = 8;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);

	return 0;
}


int16_t SendCANMessage(CanTxMsg *msg)
{
	CanTxMsg *msgBuf;
	uint8_t txResult = CAN_Transmit(CAN1, msg);
	if(CAN_TxStatus_NoMailBox == txResult)
	{
		// No room in queue, store message to buffer
		// Check for room in buffer
		// Get loc at store pointer + 1
		uint16_t ui16NextLoc = CANData.ui16CANTxMsgBufStore + 1;
		ui16NextLoc = ui16NextLoc & 0x1f;
		// If it is different than read loc, there is room in buffer
		if(ui16NextLoc != CANData.ui16CANTxMsgBufRead)
		{
			// Copy message to buf
			msgBuf = &CANData.CANTxMsgBuf[CANData.ui16CANTxMsgBufStore];

			msgBuf->DLC = msg->DLC;
			msgBuf->Data[0] = msg->Data[0];
			msgBuf->Data[1] = msg->Data[1];
			msgBuf->Data[2] = msg->Data[2];
			msgBuf->Data[3] = msg->Data[3];
			msgBuf->Data[4] = msg->Data[4];
			msgBuf->Data[5] = msg->Data[5];
			msgBuf->Data[6] = msg->Data[6];
			msgBuf->Data[7] = msg->Data[7];
			msgBuf->ExtId = msg->ExtId;
			msgBuf->IDE = msg->IDE;
			msgBuf->RTR = msg->RTR;
			msgBuf->StdId = msg->StdId;

			// Check - was buffer empty?
			if(CANData.ui16CANTxMsgBufStore == CANData.ui16CANTxMsgBufRead)
			{
				// Enable TX interrupt
				CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
			}
			CANData.ui16CANTxMsgBufStore++;
			CANData.ui16CANTxMsgBufStore = CANData.ui16CANTxMsgBufStore & 0x1f;

			return 0;
		}
		else return -1;
	}
	return 0;
}


//*********************************
// HW - independent functions

void ProcessCANMessage(CanRxMsg *msg)
{
	FLIGHT_MOTOR* FMotorData;
	uint32_t ui32ID = 0;
	uint32_t ui32Source = 0;
	ui32ID = (msg->ExtId >> 8) & 0xffff;
	ui32Source = msg->ExtId & 0x7f;

	switch(ui32Source)
	{
		case 0x12:
		{
			FMotorData = &FCFlightData.MOTORS.FR;
			break;
		}
	}

	switch(ui32ID)
	{
		case (uint32_t)CAN_MSG_ESC_RPM:
		{
			FMotorData->i16CurrentRPM = (int16_t)msg->Data[0];
			break;
		}
		default:
		{
			break;
		}
	}
}

void SendCANMessageServoAngle(uint8_t ui8ServoID, int16_t i16ReqAngle)
{
	CanTxMsg msg;
	msg.ExtId = ((uint32_t)CAN_SET_SERVO_ANGLE << 8)||((uint32_t)ui8ServoID);
	msg.Data[0] = (i16ReqAngle >> 8)&0xff;
	msg.Data[1] = i16ReqAngle&0xff;

	SendCANMessage(&msg);
}
