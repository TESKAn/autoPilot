/*
 * can.h
 *
 *  Created on: 6. jan. 2017
 *      Author: jmoc
 */

#ifndef CAN_H_
#define CAN_H_

// Define macros
// Device IDs
#define CAN_MOTOR_FR_ID			20
#define CAN_MOTOR_FL_ID			21
#define CAN_MOTOR_RR_ID			22
#define CAN_MOTOR_RL_ID			23

#define CAN_MOTOR_FR_RL_ID		24
#define CAN_MOTOR_FL_RR_ID		25

#define CAN_MOTOR_ALL_ID		26

// Message IDs
#define CAN_SET_SERVO_ENABLE
#define CAN_CLEAR_SERVO_ENABLE
#define CAN_SET_SERVO_ANGLE	1024

typedef struct tagCANSTRUCT
{
	int16_t ui16CANTxMsgBufStore;
	int16_t ui16CANTxMsgBufRead;
	CanTxMsg CANTxMsgBuf[32];

}__attribute__((aligned(4),packed)) CANSTRUCT;

extern CANSTRUCT CANData;

void InitCANLink();
void ProcessCANMessage(CanRxMsg *msg);
int16_t CAN_SendMinMaxRPM();
int16_t CAN_SendRPM(int16_t i16Dest, uint16_t ui16RPM1, uint16_t ui16RPM2);
int16_t CAN_SendOrientation();
int16_t CAN_SendNodeStatus();
int16_t SendCANMessage(CanTxMsg *msg);

#endif /* CAN_H_ */
