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
void InitCANFilter();
void ProcessCANMessage(CanRxMsg *msg);
int16_t CAN_SendMinMaxRPM();
int16_t CAN_SendRPM_single(uint16_t RPM, uint8_t ID);
int16_t CAN_SendRPM(uint16_t frontRPM, uint16_t rearRPM, uint8_t IDs);
int16_t CAN_SendENABLE(uint8_t ui8Enable, uint8_t IDs);
int16_t CAN_SendOrientation();
int16_t CAN_SendNodeStatus();
int16_t SendCANMessage(CanTxMsg *msg);

#endif /* CAN_H_ */
