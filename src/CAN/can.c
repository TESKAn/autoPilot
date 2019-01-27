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
	// Init filters to subscribe to reports
	uint32_t ui32Filter = CAN_MID_RPMINFO << 11;	// shift 8 + 3 bits
	uint32_t ui32Mask = 0xffff00 << 3;
	// Init CAN filter(s) for rx messages
	CAN_FilterInitTypeDef CANFilterInitStruct;
	CANFilterInitStruct.CAN_FilterIdHigh = (uint16_t)(ui32Filter >> 16);
	CANFilterInitStruct.CAN_FilterIdLow = (uint16_t)(ui32Filter);
	CANFilterInitStruct.CAN_FilterMaskIdHigh = (uint16_t)(ui32Mask >> 16);
	CANFilterInitStruct.CAN_FilterMaskIdLow = (uint16_t)(ui32Mask);
	CANFilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CANFilterInitStruct.CAN_FilterNumber = 0;
	CANFilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CANFilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CANFilterInitStruct.CAN_FilterActivation = ENABLE;

	CAN_FilterInit(&CANFilterInitStruct);


}


void InitCANFilter()
{
	uint16_t ui16IDHigh = (uint16_t)(ui32MainLoopCanVar >> 16);
	uint16_t ui16IDLow = (uint16_t)(ui32MainLoopCanVar);

	uint16_t ui16IDHigh1 = (uint16_t)(ui32MainLoopCanVar1 >> 16);
	uint16_t ui16IDLow1 = (uint16_t)(ui32MainLoopCanVar1);

	// Init CAN filter(s) for rx messages
	CAN_FilterInitTypeDef CANFilterInitStruct;
	CANFilterInitStruct.CAN_FilterIdHigh = ui16IDHigh; //0;//0x004e;
	CANFilterInitStruct.CAN_FilterIdLow = ui16IDLow; //0;//0x2123;
	CANFilterInitStruct.CAN_FilterMaskIdHigh = ui16IDHigh1;
	CANFilterInitStruct.CAN_FilterMaskIdLow = ui16IDLow1;
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

	// Min RPM
	cnvrt_number.i16[0] = 2000;
	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	// Max RPM
	cnvrt_number.i16[0] = 20000;
	msg.Data[2] = cnvrt_number.ch[0];
	msg.Data[3] = cnvrt_number.ch[1];

	// Destination ID - CAN_MOTOR_ALL_ID to all
	msg.Data[4] = CAN_MOTOR_ALL_ID;

	msg.Data[5] = 0xc0;
	msg.DLC = 6;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}


// IDs
// 0 -> FR_RL
// 1 -> FL_RR
int16_t CAN_SendRPM(uint16_t frontRPM, uint16_t rearRPM, uint8_t IDs)
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM, CAN_MID_SETRPM);

	// RPM 0
	cnvrt_number.ui16[0] = frontRPM;
	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	// RPM 1
	cnvrt_number.ui16[0] = rearRPM;
	msg.Data[2] = cnvrt_number.ch[0];
	msg.Data[3] = cnvrt_number.ch[1];
	// FR_RL?
	if(IDs)
	{
		msg.Data[4] = COMMData.IDs.ui8MotorFR;
		msg.Data[5] = COMMData.IDs.ui8MotorRL;
	}
	else
	{
		msg.Data[4] = COMMData.IDs.ui8MotorFL;
		msg.Data[5] = COMMData.IDs.ui8MotorRR;
	}

	msg.Data[6] = 0xc0;
	msg.DLC = 7;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendRPM_single(uint16_t RPM, uint8_t ID)
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	// Generate message ID
	switch(ID)
	{
		case CAN_MOTOR_FR_ID:
		{
			msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM_FR, CAN_MID_SETRPM_FR);
			break;
		}
		case CAN_MOTOR_FL_ID:
		{
			msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM_FL, CAN_MID_SETRPM_FL);
			break;
		}
		case CAN_MOTOR_RR_ID:
		{
			msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM_RR, CAN_MID_SETRPM_RR);
			break;
		}
		case CAN_MOTOR_RL_ID:
		{
			msg.ExtId = CAN_GenerateID(CAN_PRIO_SETRPM_RL, CAN_MID_SETRPM_RL);
			break;
		}
	}
	// RPM
	cnvrt_number.ui16[0] = RPM;
	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];

	msg.Data[2] = 0xc0;
	msg.DLC = 3;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendENABLE(uint8_t ui8Enable, uint8_t IDs)
{
	CanTxMsg msg;

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_ENABLE, CAN_MID_ENABLE);

	// RPM 0
	msg.Data[0] = ui8Enable;
	msg.Data[1] = IDs;
	msg.Data[2] = 0xc0;
	msg.DLC = 3;
	msg.IDE = CAN_Id_Extended;
	msg.RTR = CAN_RTR_Data;

	SendCANMessage(&msg);
	return 0;
}

int16_t CAN_SendOrientationPID()
{
	CONVERTNUM cnvrt_number;
	CanTxMsg msg;

	uint16_t ui16RollPID = Float32ToFloat16(FCFlightData.PIDRoll.s);
	uint16_t ui16PitchPID = Float32ToFloat16(FCFlightData.PIDPitch.s);
	uint16_t ui16YawPID = Float32ToFloat16(FCFlightData.PIDYaw.s);

	// Generate message ID
	msg.ExtId = CAN_GenerateID(CAN_PRIO_ORIENTATION_PID, CAN_MID_ORIENTATION_PID);

	cnvrt_number.ui16[0] = ui16RollPID;
	cnvrt_number.ui16[1] = ui16PitchPID;
	cnvrt_number.ui16[2] = ui16YawPID;

	msg.Data[0] = cnvrt_number.ch[0];
	msg.Data[1] = cnvrt_number.ch[1];
	msg.Data[2] = cnvrt_number.ch[2];
	msg.Data[3] = cnvrt_number.ch[3];
	msg.Data[4] = cnvrt_number.ch[4];
	msg.Data[5] = cnvrt_number.ch[5];
	msg.Data[6] = 0xc0;
	msg.DLC = 7;
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

	CONVERTNUM cnvrtnum;

	uint32_t ui32MID = (msg->ExtId >> 8) & 0xffff;
	uint32_t ui32NID = msg->ExtId & 0x7f;

	switch(ui32MID)
	{
		case CAN_MID_RPMINFO:
		{
			switch(ui32NID)
			{
				case CAN_MOTOR_FR_ID:
				{
					FMotorData = &FCFlightData.MOTORS.FR;
					break;
				}
				case CAN_MOTOR_FL_ID:
				{
					FMotorData = &FCFlightData.MOTORS.FL;
					break;
				}
				case CAN_MOTOR_RR_ID:
				{
					FMotorData = &FCFlightData.MOTORS.RR;
					break;
				}
				case CAN_MOTOR_RL_ID:
				{
					FMotorData = &FCFlightData.MOTORS.RL;
					break;
				}
			}
			cnvrtnum.ch[0] = msg->Data[0];
			cnvrtnum.ch[1] = msg->Data[1];
			FMotorData->i16SetRPM = cnvrtnum.i16[0];

			cnvrtnum.ch[0] = msg->Data[2];
			cnvrtnum.ch[1] = msg->Data[3];
			FMotorData->i16CurrentRPM = cnvrtnum.i16[0];

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
