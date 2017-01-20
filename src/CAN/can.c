/*
 * can.c
 *
 *  Created on: 6. jan. 2017
 *      Author: jmoc
 */

#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "customTypedefs.h"
#include "can.h"

CANSTRUCT CANData;

//********************************
// Hardware - dependent
void InitCANLink()
{
	// Init CAN filter(s) for rx messages
	CAN_FilterInitTypeDef CANFilterInitStruct;
	CANFilterInitStruct.CAN_FilterIdHigh = 0;
	CANFilterInitStruct.CAN_FilterIdLow = 0;
	CANFilterInitStruct.CAN_FilterMaskIdHigh = 0;
	CANFilterInitStruct.CAN_FilterMaskIdLow = 0;
	CANFilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CANFilterInitStruct.CAN_FilterNumber = 0;
	CANFilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CANFilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CANFilterInitStruct.CAN_FilterActivation = ENABLE;

	CAN_FilterInit(&CANFilterInitStruct);

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
		ui16NextLoc = ui16NextLoc & 0xff;
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
			CANData.ui16CANTxMsgBufStore = CANData.ui16CANTxMsgBufStore & 0xff;

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
	switch(msg->ExtId)
	{
		case 0:
		{
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
