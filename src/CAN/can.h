/*
 * can.h
 *
 *  Created on: 6. jan. 2017
 *      Author: jmoc
 */

#ifndef CAN_H_
#define CAN_H_



// Message IDs
#define CAN_SET_SERVO_ENABLE
#define CAN_CLEAR_SERVO_ENABLE
#define CAN_SET_SERVO_ANGLE	1024

typedef struct tagCANSTRUCT
{
	int16_t ui16CANTxMsgBufStore;
	int16_t ui16CANTxMsgBufRead;
	int16_t ui16CANTxMsgsStored;
	CanTxMsg CANTxMsgBuf[32];
	uint32_t ui32CANRXMessages;
	uint32_t ui32CANTXMessages;

}__attribute__((aligned(4),packed)) CANSTRUCT;

extern CANSTRUCT CANData;

void InitCANLink();
void ProcessCANMessage(CanRxMsg *msg);
uint32_t CAN_GenerateID(uint32_t ui32PRIO, uint32_t ui32MID);
int16_t CAN_SendRegValue(uint16_t ui16RegNumber, T32BITVARS *t32Data, uint8_t ID);
int16_t CAN_SendMinMaxRPM();
int16_t CAN_SendVoltageCutoff(uint16_t ui16Value, uint8_t ID);
int16_t CAN_SendRPM_single(uint16_t RPM, uint8_t ID);
int16_t CAN_SendENABLE(uint8_t ui8Enable, uint8_t IDs);
int16_t CAN_SendRESET(uint8_t IDs);
int16_t CAN_SendOrientationPID();
int16_t CAN_SendOrientation();
int16_t CAN_SendNodeStatus();
int16_t SendCANMessage(CanTxMsg *msg);

#endif /* CAN_H_ */
